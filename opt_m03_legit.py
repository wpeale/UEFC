import numpy as np
from scipy.optimize import minimize
from scipy import optimize
from GetUEFC import UEFC
from calc_mO3 import calc_mO3
import UEFC_wing

# Aircraft Fixed Constants
CL_MAX = 0.8
DB_MAX = 0.08
DIHEDRAL = 8.0
TAPER = 0.4
TAU = 0.08

# MPayO3 Multivariate Optimizer Params
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

# VLM Optimizer Params
CL_DES_INITIAL = 0.8
CL_GUESS = (0.7)
CL_BOUNDS = [0, CL_MAX]


def get_specified_uefc(e0, CLdes):
    specified_uefc = UEFC(
        taper=TAPER,
        tau=TAU,
        CLdes=CLdes,
        e0=e0,
        dbmax=DB_MAX,
        dihedral=DIHEDRAL,
    )
    return specified_uefc


def opt_mO3(e0, CLdes):
    def mO3_fcn(opt_vars):
        specified_uefc = get_specified_uefc(e0, CLdes)
        mO3 = -calc_mO3(specified_uefc, opt_vars[:4], opt_vars[3], opt_vars[4]) ** (
            1.0 / 3.0
        )
        return mO3

    def T_constraint_fcn(opt_vars):
        specified_uefc = get_specified_uefc(e0, CLdes)
        constraint_value = specified_uefc.excess_thrust(
            opt_vars[:4], opt_vars[3], opt_vars[4]
        )
        return constraint_value

    def db_constraint_fcn(opt_vars):
        specified_uefc = get_specified_uefc(e0, CLdes)
        constraint_value = (
            specified_uefc.dbmax
            - specified_uefc.wing_tip_deflection(
                opt_vars[:4], opt_vars[3], opt_vars[4]
            ),
        )
        return constraint_value

    def CL_constraint_fcn(opt_vars):
        specified_uefc = get_specified_uefc(e0, CLdes)
        constraint_value = (
            specified_uefc.CLdes
            - specified_uefc.lift_coefficient(opt_vars[:4], opt_vars[3], opt_vars[4]),
        )
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
    T_constraint = {"type": "ineq", "fun": T_constraint_fcn}

    # Wingtip deflection must be less than the maximum allowed value.
    db_constraint = {"type": "ineq", "fun": db_constraint_fcn}

    # Lift coefficient must be less than the maximum allowed cruise value.
    CL_constraint = {"type": "ineq", "fun": CL_constraint_fcn}

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
        max_uefc = get_specified_uefc(e0, CLdes)
        mO3_max = calc_mO3(
            max_uefc, opt_vars_maxObj[:4], opt_vars_maxObj[3], opt_vars_maxObj[4]
        )

    else:  # If optimizer fails
        opt_vars_maxObj = np.zeros(np.size(initialGuess))
        mO3_max = 0

    return opt_vars_maxObj, mO3_max, success


def opt_vlm(uefc, AR, S, CLdes):
    wing_dims = uefc.wing_dimensions(AR, S)
    # Set-up a wing object. Inputs: wingspan, root chord, tip chord, root incidence
    # angle, tip incidence angle, dihedral angle.
    # Note: wing twist is defined as (agroot - agtip). Therefore, this wing has a
    # twist of +5 degrees, and the tip is at a lower incidence angle than the root.
    PV = UEFC_wing.UEFC_wing(
        b=wing_dims["Span"],
        croot=wing_dims["Root chord"],
        ctip=wing_dims["Tip chord"],
        agroot=3.13,
        agtip=-5.0 + 3.13,
        dihedral=DIHEDRAL,
    )

    def constraint_fcn(CL):
        # Solve the flow around the wing at a desired CL
        G, alpha = PV.solve(CL=CL)        
        AR = PV.get_AR()
        S = PV.get_S()
        cl, y = PV.calccldist(G)
        CL, CDi, e0, clmax = PV.calc_aeroperf(G)
        return clmax - CL_MAX

    # Find when constraint = 0: CL corresponding to max_cl = 0.8
    result = optimize.root_scalar(constraint_fcn, bracket=CL_BOUNDS)
    
    G, alpha = PV.solve(CL=result.root)        
    AR = PV.get_AR()
    S = PV.get_S()
    cl, y = PV.calccldist(G)
    CL, CDi, e0, clmax = PV.calc_aeroperf(G)
    
    return CL, e0


if __name__ == "__main__":
    CLdes = CL_DES_INITIAL
    e0 = 1.0

    for i in range(10):
        opt_vars_maxmO3, mO3_max, success = opt_mO3(e0, CLdes)
        max_uefc = get_specified_uefc(e0, CLdes)
        CL = max_uefc.lift_coefficient(
            opt_vars_maxmO3[:3], opt_vars_maxmO3[3], opt_vars_maxmO3[4]
        )
        print("MPayO3 Optimizer Outut:")
        print("Aspect ratio: %0.4f" % opt_vars_maxmO3[3])
        print("Wing area:    %0.4f m^2" % opt_vars_maxmO3[4])
        print("CL:    %f m^2" % CL)
        print("Load factor:  %0.3f" % opt_vars_maxmO3[0])
        print("Turn radius:  %0.2f m" % opt_vars_maxmO3[1])
        print("Payload mass: %0.0f g" % opt_vars_maxmO3[2])

        print(opt_vars_maxmO3)
        print("mpay Omega^3: %0.0f g/s^3" % mO3_max)
        print("")
        print("")

        CLdes, e0 = opt_vlm(max_uefc, opt_vars_maxmO3[3], opt_vars_maxmO3[4], CL)
        print("VLM Optimizer Output:")
        print("CL  = {:.2f}".format(CLdes))  # Maximum 2D lift coefficient
        print("e0     = {:.2f}".format(e0))  # Span efficiency in level flight
        print("")
        print("")
