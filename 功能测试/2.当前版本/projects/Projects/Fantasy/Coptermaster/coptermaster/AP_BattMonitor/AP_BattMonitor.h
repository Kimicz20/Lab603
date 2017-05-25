/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_BATTMONITOR_H
#define AP_BATTMONITOR_H

#include "../AP_Common/AP_Common.h"
#include "../AP_Param/AP_Param.h"
#include "../AP_Math/AP_Math.h"

// maximum number of battery monitors
#define AP_BATT_MONITOR_MAX_INSTANCES       1

// first monitor is always the primary monitor
#define AP_BATT_PRIMARY_INSTANCE            0

#define AP_BATT_CAPACITY_DEFAULT            3300
#define AP_BATT_LOW_VOLT_TIMEOUT_MS         10000   // low voltage of 10 seconds will cause battery_exhausted to return true

// declare backend class
class AP_BattMonitor_Backend;
class AP_BattMonitor_Analog;
class AP_BattMonitor_SMBus;
class AP_BattMonitor_SMBus_I2C;
class AP_BattMonitor_SMBus_PX4;


//FixÐÞ¸Ä2.1
/*--------------- ¸¨ÖúÀà -----------------*/
#include "../SupportClass/SupportClass.h"

class AP_BattMonitor
{
    friend class AP_BattMonitor_Backend;
    friend class AP_BattMonitor_Analog;
    friend class AP_BattMonitor_SMBus;
    friend class AP_BattMonitor_SMBus_I2C;
    friend class AP_BattMonitor_SMBus_PX4;

public:
	/*--------------- ¸¨ÖúÀà -----------------*/
	SupportClass *supt;

    /// Constructor
    AP_BattMonitor();

    // Battery monitor driver types
    enum BattMonitor_Type {
        BattMonitor_TYPE_NONE                       = 0,
        BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY        = 3,
        BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT = 4,
        BattMonitor_TYPE_SMBUS                      = 5,
        BattMonitor_TYPE_BEBOP                      = 6
    };

    // The BattMonitor_State structure is filled in by the backend driver
    struct BattMonitor_State {
        uint8_t     instance;           // the instance number of this monitor
        bool        healthy;            // battery monitor is communicating correctly
        float       voltage;            // voltage in volts
        float       current_amps;       // current in amperes
        float       current_total_mah;  // total current draw since start-up
        uint32_t    last_time_micros;   // time when voltage and current was last read
        uint32_t    low_voltage_start_ms;  // time when voltage dropped below the minimum
    };

    // Return the number of battery monitor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz
    void read();

#define _BattMonitor_STATE(instance) state[instance]

    // healthy - returns true if monitor is functioning
    bool healthy(uint8_t instance) const;
    bool healthy() const { return healthy(AP_BATT_PRIMARY_INSTANCE); }

    /// has_current - returns true if battery monitor instance provides current info
    bool has_current(uint8_t instance) const;

	//FixÐÞ¸Ä2.1
    bool has_current() const { 
		long start, end;
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("has_current", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 

		bool b = has_current(AP_BATT_PRIMARY_INSTANCE);
		if (supt->getParamValueWithNameAndKey("current_amps", "has_current") == 1)
			b = true;
		else
			b = false;
		end = clock();
		supt->setCurProcessResult("has_current", end, 2);
		supt->setCurProcessResult("has_current", (end - start), 3);

		return b; 
	}

    /// voltage - returns battery voltage in millivolts
    float voltage(uint8_t instance) const;

	//FixÐÞ¸Ä2.1
    float voltage() const { 
		long start, end;
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("voltage", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 

		float b = voltage(AP_BATT_PRIMARY_INSTANCE);

		end = clock();
		supt->setCurProcessResult("voltage", end, 2);
		supt->setCurProcessResult("voltage", (end - start), 3);

		return b;
	}

    // voltage2 - returns the voltage of the second battery (helper function to send 2nd voltage to GCS)
    float voltage2() const;

    /// current_amps - returns the instantaneous current draw in amperes
    float current_amps(uint8_t instance) const;

	//FixÐÞ¸Ä2.1
    float current_amps() const { 
		long start, end;
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("current_amps", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 

		float b = current_amps(AP_BATT_PRIMARY_INSTANCE);

		end = clock();
		supt->setCurProcessResult("current_amps", end, 2);
		supt->setCurProcessResult("current_amps", (end - start), 3);

		return b; 
	}

    /// current_total_mah - returns total current drawn since start-up in amp-hours
    float current_total_mah(uint8_t instance) const;
    float current_total_mah() const { return current_total_mah(AP_BATT_PRIMARY_INSTANCE); }

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    virtual uint8_t capacity_remaining_pct(uint8_t instance) const;
    uint8_t capacity_remaining_pct() const { return capacity_remaining_pct(AP_BATT_PRIMARY_INSTANCE); }

    /// exhausted - returns true if the battery's voltage remains below the low_voltage for 10 seconds or remaining capacity falls below min_capacity
    bool exhausted(uint8_t instance, float low_voltage, float min_capacity_mah);

	//FixÐÞ¸Ä2.1
    bool exhausted(float low_voltage, float min_capacity_mah) { 
		long start, end;
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("exhausted", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 

		bool b = exhausted(AP_BATT_PRIMARY_INSTANCE, low_voltage, min_capacity_mah);
		if (supt->getParamValueWithNameAndKey("failsafe_battery_event", "has_exhausted") == 1)
			b = true;
		end = clock();
		supt->setCurProcessResult("exhausted", end, 2);
		supt->setCurProcessResult("exhausted", (end - start), 3);

		return b;
	}

	//FixÐÞ¸Ä2.1
    /// get_type - returns battery monitor type
    enum BattMonitor_Type get_type() { 
		long start, end;
		// ------------------------  ²å×®µã ---------------------------------
		start = clock();
		supt->setCurProcessResult("get_type", start, 1);
		// ------------------------  ²å×®¼¤Àø --------------------------------- 
		
		enum BattMonitor_Type b = get_type(AP_BATT_PRIMARY_INSTANCE);
		//FixÐÞ¸Ä2.2
		if (supt->getParamValueWithNameAndKey("voltage", "battmonitortype") == NOTFIND)
			b = BattMonitor_TYPE_NONE;
		else
			b = BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY;
		end = clock();
		supt->setCurProcessResult("get_type", end, 2);
		supt->setCurProcessResult("get_type", (end - start), 3);

		return b;
	}
    enum BattMonitor_Type get_type(uint8_t instance) { return (enum BattMonitor_Type)_monitoring[instance].get(); }

    /// set_monitoring - sets the monitor type (used for example sketch only)
    void set_monitoring(uint8_t instance, uint8_t mon) { _monitoring[instance].set(mon); }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_Int8     _monitoring[AP_BATT_MONITOR_MAX_INSTANCES];         /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8     _volt_pin[AP_BATT_MONITOR_MAX_INSTANCES];           /// board pin used to measure battery voltage
    AP_Int8     _curr_pin[AP_BATT_MONITOR_MAX_INSTANCES];           /// board pin used to measure battery current
    AP_Float    _volt_multiplier[AP_BATT_MONITOR_MAX_INSTANCES];    /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float    _curr_amp_per_volt[AP_BATT_MONITOR_MAX_INSTANCES];  /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float    _curr_amp_offset[AP_BATT_MONITOR_MAX_INSTANCES];    /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Int32    _pack_capacity[AP_BATT_MONITOR_MAX_INSTANCES];      /// battery pack capacity less reserve in mAh

private:
    BattMonitor_State state[AP_BATT_MONITOR_MAX_INSTANCES];
    AP_BattMonitor_Backend *drivers[AP_BATT_MONITOR_MAX_INSTANCES];
    uint8_t     _num_instances;                                     /// number of monitors
};
#endif  // AP_BATTMONITOR_H
