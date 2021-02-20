#include "stdlibpatch.h"

#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling

#include <type_traits>
#include <atomic>
#include <limits>

#include <unifex/receiver_concepts.hpp>
#include <unifex/sender_concepts.hpp>
#include <unifex/scheduler_concepts.hpp>

#include "manual_loop_context.hpp"
#include "timer_context.hpp"
#include "sequence.hpp"
#include <unifex/just.hpp>
#include <unifex/on.hpp>

//********************** PHYSICAL PARAMETERS **************************
const float beam_width_mm = 15.0f;                                       // defined by the IRPS hardware geometry
const float beam_width_um = beam_width_mm * 1000.0f;                     // a convenient float version of beam_width_mm to beam_width_mm (micrometers) to reduce calculation time
const float beam_center_mm = beam_width_mm / 2.0f;                       // halfway through an IRPS sensor
const float beam_center_um = beam_center_mm * 1000.0f;                   // a convenient float version of beam_center_mm to beam_center_mm (micrometers) to reduce calculation time
const float interbeam_dist_mm = 30.0f;                                   // defined by the IRPS hardware geometry
const float interbeam_dist_um = interbeam_dist_mm * 1000.0f;             // a convenient float version of interbeam_dist_mm to interbeam_dist_um (micrometers) to reduce calculation time
const float beam2em_dist_mm = 24.0f;                                     // defined by the length of the connecting rod between IRPS and EM
const float beam2em_dist_um = beam2em_dist_mm * 1000.0f;                 // a convenient float version of beam2em_dist_mm to beam2em_dist_um (micrometers) to reduce calculation time
const float em_width_mm = 15.0f;                                         // defined by width of EM
const float em_width_um = em_width_mm * 1000.0f;                         // a convenient float version of em_width_mm to em_width_um (micrometers) to reduce calculation time
const float em_center_mm = em_width_mm / 2.0f;                           // halfway through the EM
const float em_center_um = em_center_mm * 1000.0f;                       // a convenient float version of em_center_mm to em_center_um (micrometers) to reduce calculation time

const float max_pulse_dist_mm = 
  beam_center_mm + beam2em_dist_mm + em_center_mm;                       // all pulses will be relative to this max distance
const float max_pulse_dist_um = max_pulse_dist_mm * 1000.0f;             // all pulses will be relative to this max distance

const float clockHz = 16000000.0f;
const int timer_prescaler = 8;
const int timer_prescaler_mask = (0 << CS12) | (1 << CS11) | (0 << CS10);
const float clicks_per_second = clockHz / timer_prescaler;
const float clicks_per_us = clicks_per_second / 1000000.0f;              // eg. with an Arduino Uno (16Mhz clk/8), use 2 processor clicks per microsecond
const unsigned long em_max_safe_clicks = 65536;                          // 16bit clock
//*********************************************************************

//********************** TIMER ****************************************
using arduino_clock = typename unifex::timer_context::clock_t;
using arduino_clicks = typename arduino_clock::duration;
unifex::timer_context timer;
//*********************************************************************

//********************** SCREEN ***************************************
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 8
#define TFT_CS 10
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
//*********************************************************************

//********************** PID ******************************************
#include "PID_v2.h"
// Specify the links and initial tuning parameters
//double Kp = 4, Ki = 0.2, Kd = 0.1;
const float max_pid_em_output_um = max_pulse_dist_um * 0.6f;
const float min_pid_em_output_um = 800.0f;
// Ziegler–Nichols method for tuning PID
double Ku = (max_pid_em_output_um * 0.50f) - min_pid_em_output_um, Tu = 2.00f; 
double Kp = (Ku * 0.60f), Ki = 1.2f * Ku / Tu, Kd = 3.0f * Ku * Tu / 40.00f;
PID_v2 ppaPID(Kp, Ki, Kd, PID::Direct);
//*********************************************************************

//********************** THROTTLE PARAMETERS **************************
//throttle variables
int cv = 5;     //correction value
//Set these values manually.  They are derived in parts in range 0-1023
const int low_no_switch = 458 - cv;
const int high_no_switch = 852 + cv;
//*********************************************************************

double updateThrottle(const int pin) {
  float throttle = analogRead(pin);
  if (throttle < low_no_switch) {
    return 1.50f;
  } else if (throttle > high_no_switch) {
    return 8.0f;
  } else {
    // map from throttle position to 1.50m/s - 8.00m/s
    return map(throttle, low_no_switch, high_no_switch, 150, 800) / 100.0;
  }
}

//IR sensors and EM's
//NOTE: in board 11 onwards, OUT_FLAG is pulsed LOW to activate electromagnets.
//set via polarity definitions below
#define EM_ACTIVE LOW
#define EM_DISABLE HIGH
#include <QueueArray.h>

volatile int irps_number = 0;             // holds the current IRPS being triggered

//Screen and reporting variables
int xPos = 0;         // the x-position of the rolling velocity graph, which is incremented as the graph covers the screen
int last_irps_number = 0;

//*************************** PINOUT PARAMETERS ******************************************
//Pinouts - see 'arduino pin selection.jpg'
//        serial_pc     =0;
//        serial_pc     =1;
const int ir_interrupt  = 2;
const int ir_add0       = 3;
const int ir_add1       = 4;
const int ir_add2       = 5;
const int out_flag      = A3;
const int out_add0      = A2;
const int out_add1      = A1;
const int out_add2      = A0;
const int display_dc    = TFT_DC;
const int display_cs    = TFT_CS;
const int display_sda   = 11;
const int display_sc    = 13;
const int thr           = A4;
//const int scope0_toggle = A5;
//const int da            =18;    //A4   I2C for BMP280
//const int cl            =19;    //A5   I2C for BMP280
//***************************************************************************************

// create a queue of characters for reporting over USB
QueueArray <char> printqueue;

enum class IRPSStatus {
  Ready = 0,
  Entered = 1,
  Exited = 2 
};

// The main structure to hold an upcoming EM activation
struct ir_event {
  int irps_number;
  int em_number;
  arduino_clock::time_point startTime; // when the EM is activated
  arduino_clock::time_point endTime;   // when the EM is deactivated
  String print_string;                 // the reporting string to be sent over USB
  IRPSStatus irps_status;              // indicates prior first beam break (0), after first beam break (1), and after second beam break (2)
};

//Infra-Red Position Sensor (IRPS)
class IRPS
{
    // Class Member Variables
    // These are initialized at startup
    int irps_number_;          // the number of the IRPS
    int em_number_;            // the em related to the IRPS
    bool enabled_;             // flag indicates whether beam break events are further processed into an EM activation
    int interbeam_dist_mm_;    // mm distance between beams
    int max_dist_mm_;          // mm distance from closest beam to centre of beam to em distance
    int pulse_dist_mm_;        // mm distance from closest beam to centre of em
    bool send_to_print_stack_; // flag indicates whether beam break events are printed to stack for subsequent sending over USB

    // Store the current state
    IRPSStatus irps_status_;        // status of IRPS
    arduino_clock::time_point s1_;  // s1_us is when first beam is interrupted
    arduino_clock::time_point s2_;  // s2_us is when second beam is interrupted

    // Constructor - creates a IRPS
    // and initializes the member variables and state
  public:
    IRPS(int irps_number, int em_number, bool enabled, int interbeam_dist_mm, int max_dist_mm, bool send_to_print_stack)
    {
      irps_number_ = irps_number;
      em_number_ = em_number;
      enabled_ = enabled;
      interbeam_dist_mm_ = interbeam_dist_mm;
      max_dist_mm_ = max_dist_mm;
      send_to_print_stack_ = send_to_print_stack;
      pulse_dist_mm_ = 0;
      reset();
    }

  public:
    static int readIrpsAddress() {
      auto irps_port = portInputRegister(digitalPinToPort(ir_add0));
      static const uint8_t irps_bitmask = digitalPinToBitMask(ir_add0) | digitalPinToBitMask(ir_add1) | digitalPinToBitMask(ir_add2);
      return (~*irps_port & irps_bitmask) >> 3;
    }

    void reset() {
      irps_status_ = IRPSStatus::Ready;
      s1_ = s2_ = arduino_clock::time_point{};
    }

    void set_pulse_dist_mm(int pulse_mm) {
      pulse_dist_mm_ = std::min(pulse_mm, max_dist_mm_);
    }
    
    IRPSStatus get_status() {
      return irps_status_;
    }

    int get_irps_number() {
      return irps_number_;
    }

    int get_em_number() {
      return em_number_;
    }
    
    // microseconds between first and second beam breaks
    arduino_clicks get_delta_time() {
      if (irps_status_ == IRPSStatus::Exited && s1_ != arduino_clock::time_point{} && s2_ != arduino_clock::time_point{}) {
        return s2_ - s1_;
      }
      return arduino_clicks{-1.0f}; // invalid
    }
    float get_delta_time_us() {
      return std::chrono::duration_cast<std::chrono::microseconds>(get_delta_time()).count();
    }

    // The main function that is called when a beam interrupt occurs
    ir_event handleStatus(int irps_number)
    {

      //    first infrared beam has been broken in a valid state
      if (irps_number == irps_number_ && irps_status_ == IRPSStatus::Ready && s1_ == arduino_clock::time_point{} && s2_ == arduino_clock::time_point{}) {
        s1_ = arduino_clock::now();         // record beam break time
        irps_status_ = IRPSStatus::Entered; // update status to show first beam break
        String report;                      // hold data for printing
        if (send_to_print_stack_ == true) {
//          report = "<";
        }
        ir_event temp = {irps_number_ , em_number_, arduino_clock::time_point{}, arduino_clock::time_point{}, report, irps_status_};  // return IRPS and EM, no need to return wait, pulse or report when first beam has been broken
        return temp;
      }

      //    second infrared beam has been broken in a valid state
      else if (irps_number == irps_number_ && irps_status_ == IRPSStatus::Entered && s1_ != arduino_clock::time_point{} && s2_ == arduino_clock::time_point{}) {
        s2_ = arduino_clock::now();          // record beam break time
        irps_status_ = IRPSStatus::Exited;   // update status to show second beam break

        const auto delta_clicks_ = get_delta_time();             //time between first and second beam breaks
        const auto clicks_mm_ = delta_clicks_ / interbeam_dist_mm_;  //convert clicks to clicks per mm


        // reset when irps measurment invalid
        if (delta_clicks_.count() < 0.0f) {
          reset();
          String report;          // hold data for printing
          if (send_to_print_stack_ == true) {
            report = "r" + String(irps_number);
          }
          ir_event temp = {irps_number_ , em_number_, arduino_clock::time_point{}, arduino_clock::time_point{}, report, irps_status_};  // return IRPS and EM, no need to return wait, pulse or report when resetting
          return temp;
        }

        arduino_clock::time_point start_time_;
        arduino_clock::time_point end_time_;

        //  if the electromagnet is enabled for pulsing, calculate required timing
        if (enabled_ && pulse_dist_mm_ < max_dist_mm_) {
          start_time_ = s2_ + std::min(clicks_mm_ * (max_dist_mm_ - pulse_dist_mm_) * 1.00f, arduino::max_clicks);   // when EM pulse starts
          end_time_ = s2_ + std::min(clicks_mm_ * max_dist_mm_ * 0.97f, arduino::max_clicks);                        // when EM pulse ends
        }

        String report;          // hold data for printing
        if (send_to_print_stack_ == true) {
          //     the following calculation could be moved out of interrupt time
//          float speed_ = (float)interbeam_dist_mm / (delta_time_us_ / 1000.0f);   //calculate speed as float for reporting
//          report = String(irps_number_) + ", " + String(speed_, 2);
        }

        ir_event temp = {irps_number_ , em_number_, start_time_, end_time_, report, irps_status_};
        return temp;
      }

      // reset
      String report;          // hold data for printing
      if (send_to_print_stack_ == true && irps_number_ == irps_number) {
        // only report invalid events that are targeted to this irps - other 
        // events are valid resets for this irps
        report = "b" + String(irps_number) + "," + String(int(irps_status_));
      }
      reset();
      ir_event temp = {irps_number_ , em_number_, arduino_clock::time_point{}, arduino_clock::time_point{}, report, irps_status_};  // return IRPS and EM, no need to return wait, pulse or report when
      return temp;
    }
   
};

//Intialise Infra-Red Position Sensors and ElectroMagnets
//**************************** IRPS PHYSICAL CONFIG PARAMETERS ***************************
//IRPS(int irps_number,int em_number,bool enabled,int interbeam_dist_mm, int beam2em_dist_mm, bool send_to_print_stack)
//note: array below allows for Super size, but will also work for Standard PPA
IRPS irps[] = {
  IRPS(0, 7, false, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(1, 0, true, interbeam_dist_mm, max_pulse_dist_mm, true),
  IRPS(2, 2, true, interbeam_dist_mm, max_pulse_dist_mm, true),
  IRPS(3, 1, true, interbeam_dist_mm, max_pulse_dist_mm, true),
  IRPS(4, 4, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(5, 5, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(6, 3, true, interbeam_dist_mm, max_pulse_dist_mm, false)
};
//~,1,0,2,4,5,3
//***************************************************************************************
const int irpsCount = sizeof(irps) / sizeof(IRPS);

static class IRPS& active_irps() {
  return irps[irps_number];
}

enum class EMStatus {
  Off = 0,
  Pending = 1,
  On = 2 
};

class EM
{
    struct timer_complete {
      EM& em_;
      ir_event fire_;

      auto& get_timer_op() {
        return em_.timer_op_;
      }
    
      template<typename... Vn>
      friend void tag_invoke(unifex::tag_t<unifex::set_value>, timer_complete&& self, Vn&&...) {
        timer_complete next{std::move(self)};
        
        next.get_timer_op().destruct();

        if (next.em_.get_status() == EMStatus::Pending) {
          next.em_.startPulse(std::move(next.fire_));
        }
        else if (next.em_.get_status() == EMStatus::On) {
          next.em_.stopPulse();
        } 
      }
    
      template<typename Error>
      friend void tag_invoke(unifex::tag_t<unifex::set_error>, timer_complete&& self, Error&& error) noexcept {
        self.em_.stopPulse();
        self.get_timer_op().destruct();
      }
      friend void tag_invoke(unifex::tag_t<unifex::set_done>, timer_complete&& self) noexcept {
        self.em_.stopPulse();
        self.get_timer_op().destruct();
      }
    };
    friend struct timer_complete;
    // Class Member Variables, initialized at startup
    int em_number_;   // the number of the EM
    bool enabled_;    // flag indicates whether the EM can be turned on
    EMStatus status_;       // EM - pending, on, off
    unsigned long safetime_;    // the maximum allowed pulse duration
    unifex::manual_lifetime<unifex::connect_result_t<
      unifex::schedule_at_result_t<std::decay_t<
        unifex::get_scheduler_result_t<unifex::timer_context&>>&,
          const typename arduino_clock::time_point&>,
      timer_complete>> timer_op_;

    // These maintain the current state
    // Constructor - creates an EM
    // and initializes the member variables and state
  public:
    EM(int em_number, bool enabled, unsigned long safetime)
    {
      em_number_ = em_number;
      enabled_ = enabled;
      safetime_ = safetime;
    }

    arduino::steady_clock::time_point waitTime_;
    arduino::steady_clock::time_point pulseTime_;

  public:

    static int readEmAddress() {
      auto em_port = portInputRegister(digitalPinToPort(out_add0));
      static const uint8_t em_bitmask = digitalPinToBitMask(out_add0) | digitalPinToBitMask(out_add1) | digitalPinToBitMask(out_add2);
      auto result = *em_port;
      // reverse bottom four bits
      result = ((result & B00001100) >> 2) | ((result & B00000011) << 2);
      result = ((result & B00001010) >> 1) | ((result & B00000101) << 1);
      return result >> 1; // remove fourth bit to leave the three bit address
    }

    void reset() {
    }

    EMStatus get_status() {
      return status_;
    }
    void set_status(EMStatus newStatus) {
      status_ = newStatus;
    }

    void setup_timer(ir_event fire) {
      stopPulse();   // ensure EM is OFF during waiting period

      waitTime_ = fire.startTime;
      pulseTime_ = fire.endTime;


//      digitalWrite(scope0_toggle, LOW);     // display wait period start on scope
      
      //    write EM output address prior to energising output line
      digitalWrite(out_add0, em_number_ & 1);
      digitalWrite(out_add1, em_number_ & 2);
      digitalWrite(out_add2, em_number_ & 4);

      if (fire.startTime > arduino_clock::now()) {
        stopPulse();
        set_status(EMStatus::Pending);          // Set EM is in waiting period 
        //    Set up timer to wait for right time to start the pulse
        auto& timerOp = timer_op_.construct_from([&, this]() noexcept {
          return unifex::connect(unifex::schedule_at(unifex::get_scheduler(timer), fire.startTime), timer_complete{*this, fire});
        });
        unifex::start(timerOp);
      } else {
        startPulse(std::move(fire));
      }
    }

    void startPulse(ir_event fire) {
//    digitalWrite(scope0_toggle, HIGH);    // display wait period end on scope

      digitalWrite(out_flag, EM_ACTIVE);    // enable the EM
      set_status(EMStatus::On);             // Set EM is now energised
      //    Set up timer to wait for right time to stop the pulse
      auto& timerOp = timer_op_.construct_from([&, this]() noexcept {
        return unifex::connect(unifex::schedule_at(unifex::get_scheduler(timer), fire.endTime), timer_complete{*this, fire});
      });
      unifex::start(timerOp);
    }

    void stopPulse() {
      digitalWrite(out_flag, EM_DISABLE);   // disable EM
      set_status(EMStatus::Off);            // clear status to show EM is off
    }

};    //end of EM class

//***************************** EM CONFIG PARAMETERS ************************************
//max pulse duration in microseconds
//(int em_number,bool output_to_em,int safetime)
//note: array below allows for Super size, but will also work for Standard PPA
EM em[] = {
  EM(0, true, em_max_safe_clicks),
  EM(1, true, em_max_safe_clicks),
  EM(2, true, em_max_safe_clicks),
  EM(3, true, em_max_safe_clicks),
  EM(4, true, em_max_safe_clicks),
  EM(5, true, em_max_safe_clicks),
};
//***************************************************************************************
const int emCount = sizeof(em) / sizeof(EM);

static class EM& active_em() {
  return em[active_irps().get_em_number()];
}

unifex::manual_loop_context loop_context;

struct receiver {
  template <typename Index, typename... Values>
  friend void tag_invoke(unifex::tag_t<unifex::set_index>, const receiver&, Index&&, Values&&...) noexcept {
  }
  friend void tag_invoke(unifex::tag_t<unifex::set_end>, receiver&&) noexcept {
  }

  template<typename... Vn>
  friend void tag_invoke(unifex::tag_t<unifex::set_value>, receiver&&, Vn&&...) {
  }

  template<typename Error>
  friend void tag_invoke(unifex::tag_t<unifex::set_error>, receiver&&, Error&& error) noexcept {
  }
  friend void tag_invoke(unifex::tag_t<unifex::set_done>, receiver&&) noexcept {
  }
  template <typename Cpo, typename... UVn>
  friend void tag_invoke(unifex::tag_t<unifex::unwound>, const receiver&, Cpo, UVn&&...) noexcept {
  }
};

//auto gen_op = unifex::connect(unifex::generate(1, 42) | unifex::transform([](int){}) | unifex::on(loop_context.get_scheduler()), receiver{});
//auto start = arduino_clock::now() + std::chrono::milliseconds(200);
//auto timer_op = unifex::connect(  
//  unifex::interval(start, std::chrono::milliseconds(100)) 
//  | unifex::transform([&](auto tick){
//      Serial.println(int(std::chrono::duration_cast<std::chrono::milliseconds>(tick-start).count()));
//    }) 
//  | unifex::on(timer.get_scheduler()), receiver{});

void setup() {

//  unifex::start(gen_op);

  Serial.begin (115200);                      // start serial communication.

//  unifex::start(timer_op);

  // high baud rate required to send data in limited time
  pinMode(ir_interrupt, INPUT);
  pinMode(ir_add0, INPUT);
  pinMode(ir_add1, INPUT);
  pinMode(ir_add2, INPUT);
  pinMode(out_flag, OUTPUT);
  pinMode(out_add0, OUTPUT);
  pinMode(out_add1, OUTPUT);
  pinMode(out_add2, OUTPUT);
  pinMode(display_dc, OUTPUT);
  pinMode(display_cs, OUTPUT);
  pinMode(display_sda, OUTPUT);
  pinMode(display_sc, INPUT);
//  pinMode(scope0_toggle, OUTPUT);

//  digitalWrite(scope0_toggle, LOW);       // initialise with turned off value
  digitalWrite(out_flag, EM_DISABLE);       // initialise with turned off value

  ppaPID.SetOutputLimits(min_pid_em_output_um, max_pid_em_output_um); // PID is used to modulate how long the pulse is in micrometers
  ppaPID.Start(0.0f, 0.0f, updateThrottle(thr));       // 

  //SCREEN
  tft.begin();
  tft.setRotation(3);
  tft.setTextWrap(false);
  tft.setAddrWindow(0, 0, tft.width(), tft.height());
  display_main();

  printqueue.setPrinter(Serial);
  printqueue.enqueue('\0');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('\0');

  irps_number = 0;
  last_irps_number = 0;

  // reset all irps
  for (int i = 0; i < irpsCount; i++) {
    irps[i].reset();
    irps[i].set_pulse_dist_mm(max_pulse_dist_mm);
  }

  // reset all em
  for (int i = 0; i < emCount; i++) {
    em[i].reset();
  }

  attachInterrupt(digitalPinToInterrupt(2), demuxINT, RISING);     // define interrupt based on rising edge of pin 2

}

//MAIN LOOP
void loop() {
  wdt_enable(WDTO_500MS);

  loop_context.run_once();

  auto& irps_ = active_irps();
  auto& em_ = active_em();

  // Output to serial connection
  if (irps_.get_irps_number() == 0 && irps_.get_status() != IRPSStatus::Entered && em_.get_status() == EMStatus::Off) {
    while (!printqueue.isEmpty ()) {
      char next = printqueue.dequeue();
      if (next != '\0') {
        Serial.print(next);
      } else {
        Serial.println("");
      }
    }
  }

  if (irps_.get_status() != IRPSStatus::Exited || em_.get_status() != EMStatus::Off) {
    return;
  }

  // update once after each EM pulse
  if (last_irps_number != irps_.get_irps_number()) {
    last_irps_number = irps_.get_irps_number();

    float delta_time_us = irps_.get_delta_time_us();

    const double target_speed = updateThrottle(thr);

    ppaPID.Setpoint(target_speed);

    if (delta_time_us > 0) {
      const double speed = interbeam_dist_um / delta_time_us;  // micrometers divided by microseconds is equivalent to metres per second

      const double output_mm = ppaPID.Run(speed) / 1000.0f;

      irps_.set_pulse_dist_mm(output_mm);

//      const float delta_clicks_ = delta_time_us * clicks_per_us;   //convert microseconds to clicks
//      const float clicks_mm_ = delta_clicks_ / interbeam_dist_mm;  //convert clicks to clicks per mm

      const auto yPos = map(speed * 100, 0, 1000, tft.height(), 49);  // scale actual speed (0.0-10.0 m/s) according to graph area on scale for plotting purposes
      const auto yTargetPos = map(target_speed * 100, 0, 1000, tft.height(), 49);  // scale target speed (0.0-10.0 m/s) according to graph area on scale for plotting purposes
  
      //Manage screen scrolling
      xPos = xPos + 1;
      if (xPos > tft.width()) {
        tft.fillRect(0, 49, tft.width(), tft.height() - 49, ILI9341_BLACK);
        xPos = 0;
      }
  
      tft.drawPixel( xPos, yPos, ILI9341_WHITE );
      tft.drawPixel( xPos, yPos - 1, ILI9341_WHITE );
      tft.drawPixel( xPos, yTargetPos, ILI9341_RED );

      tft.setCursor(0, 35);
      tft.print(speed, 2);
      tft.print(" - ");
      tft.print(target_speed, 2);
      tft.print(" - ");
      tft.print(output_mm, 2);
      tft.print("mm ");
//      tft.print(" - ");
//      tft.print(clicks_mm_ * output_mm, 2);
//      tft.print("cl ");
    }
  }
}   //end of main loop

void demuxINT() {
 
  //demultiplexes hardware ir_interrupt and calls relevant Speed Sensor (IRPS) based on address

  irps_number = IRPS::readIrpsAddress();

  if (irps_number < 0 || irps_number >= irpsCount) {
    irps_number = 0;
    printqueue.enqueue('$');
    printqueue.enqueue('\0');
    return;
  }

  auto now = arduino_clock::now();

  // notify all irps of interrupt
  for (int i = 0; i < irpsCount; i++) {
    //call function to process IR interrupt
    struct ir_event current_event = irps[i].handleStatus(irps_number);

    // if a pulse is required, then send parameters to EM function
    if (current_event.irps_status == IRPSStatus::Exited && current_event.startTime > now ) {
      em[current_event.em_number].setup_timer(current_event);
    }

    // enqueue log output
    int string_length = current_event.print_string.length();
    if (string_length > 2) {
      for (int i = 0; i <= string_length && !printqueue.isFull(); ++i) {
        printqueue.enqueue(current_event.print_string.charAt(i));
      }
    }
  }
}

void display_main() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.println("~ PPA! ~ particle_pid_v1_6EM");
  tft.println("");
  tft.print("Actual - Target (Speeds in m/sec)");
  tft.setTextSize(2);
}
