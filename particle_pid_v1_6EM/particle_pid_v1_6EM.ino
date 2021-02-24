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
#include "interrupt_context.hpp"
#include "sequence.hpp"
#include "static_submit.hpp"
#include <unifex/just.hpp>
#include <unifex/on.hpp>

//********************** UNIFEX CONTEXTS ******************************
unifex::manual_loop_context loop_context;
unifex::interrupt_context irps_context;

using arduino_clock = typename unifex::timer_context::clock_t;
using arduino_clicks = typename arduino_clock::duration;
const unsigned long em_max_safe_clicks = arduino::max_clicks.count();
unifex::timer_context timer;
//*********************************************************************


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

const float min_speed_mps = 1.8f; // slowest speed physically maintainable
const float max_speed_mps = 6.0f; // highest speed physically attainable

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
    return min_speed_mps;
  } else if (throttle > high_no_switch) {
    return max_speed_mps;
  } else {
    // map from throttle position to min - max speed in meters-per-second
    return map(throttle, low_no_switch, high_no_switch, min_speed_mps * 100, max_speed_mps * 100) / 100.0;
  }
}

template<typename ATimeScheduler>
auto runThrottle(ATimeScheduler scheduler, const int pin) {
  // update target_speed from throttle control.
  return unifex::interval(scheduler.now() + std::chrono::milliseconds(200), std::chrono::milliseconds(250))
  | unifex::transform([pin](auto tick) {
    const double target_speed = updateThrottle(pin);
    ppaPID.Setpoint(target_speed);
    return target_speed;
  })
  | unifex::on(scheduler);
}


//IR sensors and EM's
//NOTE: in board 11 onwards, OUT_FLAG is pulsed LOW to activate electromagnets.
//set via polarity definitions below
#define EM_ACTIVE LOW
#define EM_DISABLE HIGH

volatile int irps_number = 0;             // holds the current IRPS being triggered

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

enum class IRPSStatus {
  Ready = 0,
  Entered = 1,
  Exited = 2
};

// The main structure to hold an upcoming EM activation
struct ir_event {
  int irps_number;
  int em_number;
  arduino_clock::time_point eventTime; // when the event occurred
  arduino_clock::time_point startTime; // when the EM is activated
  arduino_clock::time_point endTime;   // when the EM is deactivated
//  String print_string;                 // the reporting string to be sent over USB
  IRPSStatus irps_status;              // indicates prior first beam break (0), after first beam break (1), and after second beam break (2)
  float speed;
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
      return arduino_clicks{ -1.0f}; // invalid
    }
    float get_delta_time_us() {
      return std::chrono::duration_cast<std::chrono::microseconds>(get_delta_time()).count();
    }

    // The main function that is called when a beam interrupt occurs
    ir_event handleStatus(int irps_number, unifex::timer_context::time_point now)
    {

      //    first infrared beam has been broken in a valid state
      if (irps_number == irps_number_ && irps_status_ == IRPSStatus::Ready && s1_ == arduino_clock::time_point{} && s2_ == arduino_clock::time_point{}) {
        s1_ = now;                                 // record beam break time
        irps_status_ = IRPSStatus::Entered;        // update status to show first beam break
//        String report;                             // hold data for printing
//        if (send_to_print_stack_ == true) {
//          report = "Entered IRPS";
//        }
        ir_event temp = {irps_number_ , em_number_, now, now, now, /*report,*/ irps_status_, 0.0f};  // return IRPS and EM, no need to return wait, pulse or report when first beam has been broken
        return temp;
      }

      //    second infrared beam has been broken in a valid state
      else if (irps_number == irps_number_ && irps_status_ == IRPSStatus::Entered && s1_ != arduino_clock::time_point{} && s2_ == arduino_clock::time_point{}) {
        s2_ = now;                           // record beam break time
        irps_status_ = IRPSStatus::Exited;   // update status to show second beam break

        const auto delta_clicks_ = get_delta_time();                 // time between first and second beam breaks
        const auto clicks_mm_ = delta_clicks_ / interbeam_dist_mm_;  // convert clicks to clicks per mm


        // reset when irps measurment invalid
        if (delta_clicks_.count() < 0.0f) {
          reset();
//          String report;          // hold data for printing
//          if (send_to_print_stack_ == true) {
//            report = "invalid delta";
//          }
          ir_event temp = {irps_number_ , em_number_, now, now, now, /*report,*/ irps_status_, 0.0f};  // return IRPS and EM, no need to return wait, pulse or report when resetting
          return temp;
        }

        arduino_clock::time_point start_time_;
        arduino_clock::time_point end_time_;

        //  if the electromagnet is enabled for pulsing, calculate required timing
        if (enabled_ && pulse_dist_mm_ <= max_dist_mm_) {
          start_time_ = s2_ + std::min(clicks_mm_ * (max_dist_mm_ - pulse_dist_mm_) * 1.00f, arduino::max_clicks);   // when EM pulse starts
          end_time_ = s2_ + std::min(clicks_mm_ * max_dist_mm_ * 1.0f, arduino::max_clicks);                        // when EM pulse ends
        }

        constexpr auto clicks_per_second = std::chrono::duration_cast<arduino_clicks>(std::chrono::seconds(1));
        float meters_per_second_ = clicks_per_second / (clicks_mm_ * 1000.0f);   //calculate speed as float for reporting

//        String report;          // hold data for printing
//        if (send_to_print_stack_ == true) {
//          report = "Exited IRPS";
//        }

        ir_event temp = {irps_number_ , em_number_, now, start_time_, end_time_, /*report,*/ irps_status_, meters_per_second_};
        return temp;
      }

      // reset
//      String report;          // hold data for printing
//      if (send_to_print_stack_ == true && irps_number_ == irps_number) {
//        // only report invalid events that are targeted to this irps - other
//        // events are valid resets for this irps
//        report = "invalid state";
//      }
      reset();
      ir_event temp = {irps_number_ , em_number_, now, now, now, /*report,*/ irps_status_, 0.0f};  // return IRPS and EM, no need to return wait, pulse or report when
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
//  IRPS(4, 4, true, interbeam_dist_mm, max_pulse_dist_mm, false),
//  IRPS(5, 5, true, interbeam_dist_mm, max_pulse_dist_mm, false),
//  IRPS(6, 3, true, interbeam_dist_mm, max_pulse_dist_mm, false)
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

    arduino_clock::time_point waitTime_;
    arduino_clock::time_point pulseTime_;

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

      if (fire.startTime > fire.eventTime) {
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
//  EM(3, true, em_max_safe_clicks),
//  EM(4, true, em_max_safe_clicks),
//  EM(5, true, em_max_safe_clicks),
};
//***************************************************************************************
const int emCount = sizeof(em) / sizeof(EM);

static class EM& active_em() {
    return em[active_irps().get_em_number()];
}

double updatePID(const struct ir_event& current_event) {
  const double output_mm = ppaPID.Run(current_event.speed) / 1000.0f;

  for (int irpsIndex = 0; irpsIndex < irpsCount; ++irpsIndex) {
    irps[irpsIndex].set_pulse_dist_mm(output_mm);
  }

  return output_mm;
}

void drawEvent(unifex::timer_context::time_point now, const struct ir_event& current_event) {
  const auto tick_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_event.eventTime.time_since_epoch());
  const double target_speed = ppaPID.GetSetpoint();

  constexpr auto sixty_seconds = 60'000;
  const auto xPos_ = map(tick_ms.count() % sixty_seconds, 0, sixty_seconds, 0, tft.width());  // scale time according to graph area on scale for plotting purposes
  const auto yPos = map(current_event.speed * 100, 0, 800, tft.height(), 52);  // scale actual speed according to graph area on scale for plotting purposes
  const auto yTargetPos = map(target_speed * 100, 0, 800, tft.height(), 52);  // scale target speed according to graph area on scale for plotting purposes

  // clear next line
  tft.drawFastVLine(xPos_, 52, tft.height() - 52, ILI9341_BLACK);
  tft.drawFastVLine(xPos_ + 1, 52, tft.height() - 52, ILI9341_BLACK);
  tft.drawFastVLine(xPos_ + 2, 52, tft.height() - 52, ILI9341_BLACK);

  // indicator for current position
  tft.drawFastHLine(0, tft.height() - 2, tft.width(), ILI9341_BLACK);
  tft.drawFastHLine(0, tft.height() - 1, tft.width(), ILI9341_BLACK);
  tft.drawFastVLine( xPos_, tft.height() - 2, 2, ILI9341_GREEN );

  // actual speed
  tft.drawFastVLine( xPos_, yPos, 2, ILI9341_WHITE );

  // target speed
  tft.drawPixel( xPos_, yTargetPos, ILI9341_RED );

  tft.setCursor(0, 35);
  tft.print(current_event.speed, 2);
  tft.print(" - ");
  tft.print(target_speed, 2);
  tft.print(" - ");
  using days_t = std::chrono::duration<std::intmax_t, std::ratio<86400>>;
  const auto days = std::chrono::duration_cast<days_t>(tick_ms);
  const auto hours = std::chrono::duration_cast<std::chrono::hours>(tick_ms) % std::chrono::hours(12);
  const auto minutes = std::chrono::duration_cast<std::chrono::minutes>(tick_ms) % std::chrono::minutes(60);
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(tick_ms) % std::chrono::seconds(60);
  tft.print((long)days.count());
  tft.print(":");
  tft.print((long)hours.count());
  tft.print(":");
  tft.print((long)minutes.count());
  tft.print(":");
  tft.print((long)seconds.count());
}

void printEvent(double output_mm, unifex::timer_context::time_point now, const struct ir_event& current_event) {
  const auto tick_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_event.eventTime.time_since_epoch());

  using seconds_t = std::chrono::duration<float>;
  Serial.print(std::chrono::time_point_cast<seconds_t>(now).time_since_epoch().count(), 2);
  Serial.print("s - ");
  Serial.print(std::chrono::time_point_cast<seconds_t>(current_event.eventTime).time_since_epoch().count(), 2);
  Serial.print("s - ");
  Serial.print(std::chrono::time_point_cast<seconds_t>(current_event.startTime).time_since_epoch().count(), 2);
  Serial.print("s - ");
  Serial.print(std::chrono::time_point_cast<seconds_t>(current_event.endTime).time_since_epoch().count(), 2);
  Serial.print("s - ");
  Serial.print((long)std::chrono::duration_cast<std::chrono::microseconds>(now - (current_event.endTime + std::chrono::microseconds(10))).count());
  Serial.print("us, ");
  Serial.print(current_event.irps_number);
  Serial.print(", ");
  Serial.print(output_mm);
  Serial.println("mm");
}

template <typename Disambiguator>
void queueEventOnLoop(const struct ir_event& current_event) {
  unifex::static_submit(
    unifex::schedule_at(unifex::get_scheduler(timer), current_event.endTime + std::chrono::microseconds(10)) 
    | unifex::transform([current_event, dis = Disambiguator{}](){
      unifex::static_submit(
        unifex::schedule(unifex::get_scheduler(loop_context)) 
        | unifex::transform([current_event, dis](){
          const auto now = unifex::get_scheduler(timer).now();

          if (current_event.irps_number == 2) {
            drawEvent(now, current_event);
          }

          const double output_mm = updatePID(current_event);

          printEvent(output_mm, now, current_event);
        }));
    }));


//          unifex::static_submit(
//            unifex::schedule(unifex::get_scheduler(loop_context)) 
//            | unifex::transform([current_event](){
//              const auto now = unifex::get_scheduler(timer).now();
//              using seconds_t = std::chrono::duration<float>;
//              Serial.print(std::chrono::time_point_cast<seconds_t>(now).time_since_epoch().count(), 2);
//              Serial.print("s - ");
//              Serial.print(std::chrono::time_point_cast<seconds_t>(current_event.eventTime).time_since_epoch().count(), 2);
//              Serial.print("s - ");
//              Serial.print(current_event.irps_number);
//              Serial.print(", ");
//              Serial.println(current_event.irps_status == IRPSStatus::Exited ? "Exited" : (current_event.irps_status == IRPSStatus::Entered) ? "Entered" : "Ready" );
//              Serial.print(", ");
//              Serial.println(current_event.print_string);
//            }));
}

template<typename InterruptsSender>
auto runIrps(InterruptsSender&& interruptsSender) {
    return std::forward<InterruptsSender>(interruptsSender)
    | unifex::transform([](auto index){
      //demultiplexes hardware ir_interrupt and calls relevant Speed Sensor (IRPS) based on address
  
      irps_number = IRPS::readIrpsAddress();
  
      if (irps_number < 0 || irps_number >= irpsCount) {
        irps_number = 0;
        return;
      }
  
      auto now = unifex::get_scheduler(timer).now();
  
      struct ir_event current_event = irps[irps_number].handleStatus(irps_number, now);

      for (int irpIndex = 0; irpIndex < irpsCount; ++irpIndex) {
        if (irpIndex != irps_number) {
          irps[irpIndex].reset();
        }
      }
  
      if (irps_number != 0) {
        if (current_event.irps_status == IRPSStatus::Exited && current_event.speed > 0.0f) {
          // A pulse is required, send parameters to EM function
          em[current_event.em_number].setup_timer(current_event);

          if (irps_number == 1) {
            struct irps_1{};
            queueEventOnLoop<irps_1>(current_event);
          }
          else if (irps_number == 2) {
            struct irps_2{};
            queueEventOnLoop<irps_2>(current_event);
          }
          else if (irps_number == 3) {
            struct irps_3{};
            queueEventOnLoop<irps_3>(current_event);
          }
        }
      }
    });
}

void setup() {

  Serial.begin (115200);                      // start serial communication.
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

  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.println("~ PPA! ~ particle_libunifex_v1_6EM");
  tft.println("");
  tft.print("Actual - Target (Speeds in m/sec)");
  tft.setTextSize(2);

  irps_number = 0;

  // reset all irps
  for (int i = 0; i < irpsCount; i++) {
    irps[i].reset();
    irps[i].set_pulse_dist_mm(max_pulse_dist_mm);
  }

  // reset all em
  for (int i = 0; i < emCount; i++) {
    em[i].reset();
  }

  unifex::static_submit(runThrottle(unifex::get_scheduler(timer), thr));

  unifex::static_submit(runIrps(irps_context.all_interrupts()));

  irps_context.attach(ir_interrupt, RISING);
}

//MAIN LOOP
void loop() {
//  wdt_enable(WDTO_500MS);

  if (active_irps().get_irps_number() == 0 || active_irps().get_status() != IRPSStatus::Exited || active_em().get_status() != EMStatus::Off) {
    return;
  }

  loop_context.run_once();
}   //end of main loop
