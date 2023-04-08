import numpy as np
from scipy.optimize import minimize
from GetUEFC import UEFC
from calc_mO3 import calc_mO3

CL_DES = 0.85
DB_MAX = 0.08
DIHEDRAL = 8.0

N_INITIAL_GUESS = 1.1
N_LOWER_BOUND = 1.0001
N_UPPER_BOUND = 5.0

R_INITIAL_GUESS = 6.0
R_LOWER_BOUND = 0.1
R_UPPER_BOUND = 12.5

MPAY_INITIAL_GUESS = 10.0
MPAY_LOWER_BOUND = 0.01
MPAY_UPPER_BOUND = 1000.0

AR_INITIAL_GUESS = 8.5
AR_LOWER_BOUND = 5.0
AR_UPPER_BOUND = 15.0

S_INITIAL_GUESS = 0.385
S_LOWER_BOUND = 0.1
S_UPPER_BOUND = 0.7

TAPER_INITIAL_GUESS = 0.4

TAU_INITIAL_GUESS = 0.08

def get_specified_uefc(opt_vars):
    specified_uefc = UEFC(
        taper=TAPER_INITIAL_GUESS, tau=TAU_INITIAL_GUESS, CLdes=CL_DES, dbmax=DB_MAX, dihedral=DIHEDRAL
    )
    return specified_uefc


def opt_mO3():

    # YOU SHOULD NOT NEED TO CHANGE THIS FUNCTION FOR THIS PROBLEM

    # Determine the maximum objective function (payload mass x turn rate)
    # achievable for an airplane with the inputted values of AR, S.

    # Optimization variables: N, R, and mpay

    # Maximize objective: minimize negative of objective (ORIGINAL)
    # mO3_fcn = lambda opt_vars: -calc_mO3(UEFC, opt_vars, AR, S)

    # Modification (otherwise solution not always found)
    # mO3_fcn = lambda opt_vars: -calc_mO3(UEFC, opt_vars, AR, S)**(1./3.)

    def mO3_fcn(opt_vars):
        specified_uefc = get_specified_uefc(opt_vars)
        mO3 = -calc_mO3(specified_uefc, opt_vars[:4], opt_vars[3], opt_vars[4]) ** (
            1.0 / 3.0
        )
        return mO3

    def T_constraint_fcn(opt_vars):
        specified_uefc = get_specified_uefc(opt_vars)
        constraint_value = specified_uefc.excess_thrust(opt_vars[:4], opt_vars[3], opt_vars[4])
        return constraint_value

        
    def db_constraint_fcn(opt_vars):
        specified_uefc = get_specified_uefc(opt_vars)
        constraint_value = specified_uefc.dbmax - specified_uefc.wing_tip_deflection(opt_vars[:4], opt_vars[3], opt_vars[4]),
        return constraint_value
    
    def CL_constraint_fcn(opt_vars):
        specified_uefc = get_specified_uefc(opt_vars)
        constraint_value = specified_uefc.CLdes - specified_uefc.lift_coefficient(opt_vars[:4], opt_vars[3], opt_vars[4]),
        return constraint_value

    initialGuess = (
        N_INITIAL_GUESS,
        R_INITIAL_GUESS,
        MPAY_INITIAL_GUESS,
        AR_INITIAL_GUESS,
        S_INITIAL_GUESS,
    )
    bounds = (
        (N_LOWER_BOUND, N_UPPER_BOUND),
        (R_LOWER_BOUND, R_UPPER_BOUND),
        (MPAY_LOWER_BOUND, MPAY_UPPER_BOUND),
        (AR_LOWER_BOUND, AR_UPPER_BOUND),
        (S_LOWER_BOUND, S_UPPER_BOUND),
    )

    # CONSTRAINT format is different, depending on algorithm.
    method = "SLSQP"

    # Excess thrust (max - required thrust) must be positive.
    T_constraint = {
        "type": "ineq",
        "fun": T_constraint_fcn,
    }
    
    # Wingtip deflection must be less than the maximum allowed value.
    db_constraint = {
        "type": "ineq",
        "fun": db_constraint_fcn,
    }

    # Lift coefficient must be less than the maximum allowed cruise value.
    CL_constraint = {
        "type": "ineq",
        "fun": CL_constraint_fcn,
    }
    
    constraints = [T_constraint, db_constraint, CL_constraint]

    try:
        result = minimize(
            fun=mO3_fcn,
            x0=initialGuess,
            bounds=bounds,
            constraints=constraints,
            method=method,
            options={"maxiter": 40000},
        )

        success = result.success

    except:  # Optimizer failed
        result = None
        success = False

    if success:
        opt_vars_maxObj = result.x  # Variables that maximize objective
        max_uefc = get_specified_uefc(opt_vars_maxObj)
        mO3_max = calc_mO3(max_uefc, opt_vars_maxObj[:4], opt_vars_maxObj[3], opt_vars_maxObj[4])

    else:  # If optimizer fails
        opt_vars_maxObj = np.zeros(np.size(initialGuess))
        mO3_max = 0

    return opt_vars_maxObj, mO3_max, success


if __name__ == "__main__":
    opt_vars_maxmO3, mO3_max, success = opt_mO3()
    max_uefc = get_specified_uefc(opt_vars_maxmO3)
    CL = max_uefc.lift_coefficient(opt_vars_maxmO3[:4], 8.5, 0.385)#opt_vars_maxmO3[3], opt_vars_maxmO3[4])
    mO3_max = calc_mO3(max_uefc, opt_vars_maxmO3[:4], 8.5, 0.385) #opt_vars_maxObj[3], opt_vars_maxObj[4])
    print()
    print("Aspect ratio: %0.4f" % opt_vars_maxmO3[3])
    print("Wing area:    %0.4f m^2" % opt_vars_maxmO3[4])
    print("CL:    %f m^2" % CL)

    print()
    print("Load factor:  %0.3f" % opt_vars_maxmO3[0])
    print("Turn radius:  %0.2f m" % opt_vars_maxmO3[1])
    print("Payload mass: %0.0f g" % opt_vars_maxmO3[2])

    print(opt_vars_maxmO3)
    print("mpay Omega^3: %0.0f g/s^3" % mO3_max)
