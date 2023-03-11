import numpy as np
from scipy.optimize import minimize
from GetUEFC        import UEFC
from calc_mO3       import calc_mO3

def opt_mO3(UEFC, AR, S):
    
    # YOU SHOULD NOT NEED TO CHANGE THIS FUNCTION FOR THIS PROBLEM
    
    # Determine the maximum objective function (payload mass x turn rate) 
    # achievable for an airplane with the inputted values of AR, S. 
    
    # Optimization variables: N, R, and mpay
    
    # Maximize objective: minimize negative of objective (ORIGINAL)
    #mO3_fcn = lambda opt_vars: -calc_mO3(UEFC, opt_vars, AR, S) 
    
    # Modification (otherwise solution not always found)
    mO3_fcn = lambda opt_vars: -calc_mO3(UEFC, opt_vars, AR, S)**(1./3.)
    
    N_initialGuess = 1.1
    N_lowerBound   = 1.0001
    N_upperBound   = 5.0
    
    R_initialGuess = 6.0
    R_lowerBound   = 0.1
    R_upperBound   = 12.5    
    
    mpay_initialGuess = 10.
    mpay_lowerBound   = 0.01
    mpay_upperBound   = 1000.
    
    initialGuess = (N_initialGuess, R_initialGuess, mpay_initialGuess)
    bounds       = ((N_lowerBound,    N_upperBound), 
                    (R_lowerBound,    R_upperBound),
                    (mpay_lowerBound, mpay_upperBound))
    
    # Constraint format is different, depending on algorithm.
    method = "SLSQP"
    if method == "SLSQP":
        
        # Excess thrust (max - required thrust) must be positive.
        T_constraint = {
                "type": "ineq",
                "fun":  lambda opt_vars: UEFC.excess_thrust(opt_vars, AR, S)
                }
    
        # Wingtip deflection must be less than the maximum allowed value.
        db_constraint = {
                "type": "ineq",
                "fun":  lambda opt_vars: UEFC.dbmax \
                - UEFC.wing_tip_deflection(opt_vars, AR, S)
                }
    
        # Lift coefficient must be less than the maximum allowed cruise value. 
        CL_constraint = {
                "type": "ineq",
                "fun": lambda opt_vars: UEFC.CLdes \
                - UEFC.lift_coefficient(opt_vars, AR, S)
            }

    else:
        raise AttributeError("Optimization method " + method \
                             + " not recognized.")
    
    constraints = [T_constraint, db_constraint, CL_constraint]
    
    try:
        result = minimize(fun=mO3_fcn, x0=initialGuess, bounds=bounds, 
                      constraints=constraints, method=method, 
                      options={"maxiter": 400})
        
        success = result.success
    
    except:  # Optimizer failed
        result  = None
        success = False
    
    if success:
        opt_vars_maxObj = result.x  # Variables that maximize objective
        mO3_max         = calc_mO3(UEFC, opt_vars_maxObj, AR, S)
    
    else:  # If optimizer fails
        opt_vars_maxObj = np.zeros(np.size(initialGuess))
        mO3_max         = 0
    
    return opt_vars_maxObj, mO3_max, success


if __name__ == "__main__":
    
    # Simple test case. Feel free to modify this part of the file.
    aircraft = UEFC()

    AR = 11
    S  = 0.3  # m^2

    opt_vars_maxmO3, mO3_max, success = opt_mO3(aircraft, AR, S)
    
    print()
    print("Aspect ratio: %0.4f"     % AR)
    print("Wing area:    %0.4f m^2" % S)
    
    print()
    print("Load factor:  %0.3f"   % opt_vars_maxmO3[0])
    print("Turn radius:  %0.2f m" % opt_vars_maxmO3[1])
    print("Payload mass: %0.0f g" % opt_vars_maxmO3[2])
    
    print("mpay Omega^3: %0.0f g/s^3" % mO3_max)
    
