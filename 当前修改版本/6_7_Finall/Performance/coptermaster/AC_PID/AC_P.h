// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PD.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_P_H__
#define __AC_P_H__

#include "../AP_Common/AP_Common.h"
#include "../AP_Param/AP_Param.h"
#include <stdlib.h>
#include <math.h>

/// @class	AC_P
/// @brief	Object managing one P controller
class AC_P {
public:

    /// Constructor for P that saves its settings to EEPROM
    ///
    /// @note	PIs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    ///
    AC_P(
        const float &   initial_p = 0.0f)
    {
		//AP_Param::setup_object_defaults(this, var_info);
		_kp = 0;

        _kp = initial_p;
    }

    /// Iterate the P controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    ///
    /// @returns		The updated control output.
    ///
    float       get_p(float error) const;

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator() (const float p) { _kp = p; }

    // accessors
    float       kP() const { return _kp.get(); }
    void        kP(const float v) { _kp.set(v); }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Float        _kp;
};

#endif // __AC_P_H__
