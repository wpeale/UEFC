def calc_mO3(UEFC, opt_vars, AR, S):
    
    # YOU SHOULD NOT NEED TO CHANGE THIS FUNCTION FOR THIS PROBLEM
    
    # Calculate the objective function (payload mass x turn rate^3) in g/s^3
    
    mpay  = opt_vars[2]
    Omega = UEFC.turn_rate(opt_vars, AR, S)
    
    m0 = mpay * Omega ** 3.
    
    return m0
    
    

