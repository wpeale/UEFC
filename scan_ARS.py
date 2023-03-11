# YOU SHOULD NOT NEED TO CHANGE THIS FILE FOR THIS PROBLEM

# This script will search over (AR,S) to determine optimal designs for each 
# AR,S considered. Then, it plots contours of mO3, mpay, Omega, R, d/b, CL, T, 
# and Tmax. An asterick (*) is placed at the location in (AR,S) which has the 
# highest objective (mO3).  Finally, scan_ARS also prints out what the 
# performance, operating conditions, weight breakdowns, etc are for this 
# optimized aircraft.

import numpy as np
from matplotlib import pyplot as plt
from IPython    import get_ipython

from GetUEFC        import UEFC
from opt_mO3        import opt_mO3
from report_opt_mO3 import report_opt_mO3

if __name__ == "__main__":
    
    aircraft = UEFC()
    
    nAR = nS = 41
    
    ARarray = np.linspace(5,   15,  nAR)  # Aspect-ratio values
    Sarray  = np.linspace(0.1, 0.7, nS)   # Surface-area values (m^2)
    
    ARvals, Svals = np.meshgrid(ARarray, Sarray, indexing="ij")  # 2D.
    
    # initialize 2D output arrays
    mO3vals   = np.zeros((nAR, nS))  # Objective function (g/s^3)
    mpayvals  = np.zeros((nAR, nS))  # Payload mass (g)
    Omegavals = np.zeros((nAR, nS))  # Turn rate (rad/s)
    Rvals     = np.zeros((nAR, nS))  # Turn radius (m)
    CLvals    = np.zeros((nAR, nS))  # Lift coefficient (-)
    Tvals     = np.zeros((nAR, nS))  # Required thrust (N)
    Tmaxvals  = np.zeros((nAR, nS))  # Maximum thrust (N)
    dbvals    = np.zeros((nAR, nS))  # Wingtip deflection / wingspan
    Nvals     = np.zeros((nAR, nS))  # load factor
    
    # Sweep over (AR, S)
    for iAR,AR in enumerate(ARarray):
        for iS,S in enumerate(Sarray):
            
            # Determine max objective
            opt_vars, mO3, success = opt_mO3(aircraft, AR, S)
            
            if success:

                V = aircraft.flight_velocity(opt_vars, AR, S)
                
                mO3vals[iAR,iS]   = mO3
                mpayvals[iAR,iS]  = opt_vars[2]
                Omegavals[iAR,iS] = aircraft.turn_rate(opt_vars, AR, S)
                Rvals[iAR,iS]     = opt_vars[1]
                CLvals[iAR,iS]    = aircraft.lift_coefficient(opt_vars, AR, S)
                Tvals[iAR,iS]     = aircraft.required_thrust(opt_vars, AR, S)
                Tmaxvals[iAR,iS]  = aircraft.maximum_thrust(V)
                dbvals[iAR,iS]    = aircraft.wing_tip_deflection(opt_vars, AR, S)
                Nvals[iAR,iS]     = opt_vars[0]
            
        print("Completed %3.1f%% of (AR,S) scan" % (100*(iAR+1)/nAR))
    
    # Find and print the optimal point (where mO3 is maximized)
    mO3_opt           = np.max(mO3vals)
    (iAR_opt, iS_opt) = np.unravel_index(mO3vals.argmax(), mO3vals.shape)
    
    ARopt = ARvals[iAR_opt, iS_opt]
    Sopt  = Svals[iAR_opt, iS_opt]
    
    print("\n")
    print("Maximum mO3 aircraft characteristics:")
    print("----------------------------------------------")
    report_opt_mO3(aircraft, ARopt, Sopt)    
    
    
    # Plotting commands
    plt.ion()
    plt.rc('axes', axisbelow=True)
    plt.show()
    get_ipython().run_line_magic('matplotlib', 'qt')
    marker=(8,2,0)  # 8-sided asterisk
    
    # Objective function: mO3
    fig1   = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(mO3vals[mO3vals > 0]), np.max(mO3vals), 21)
    cs     = plt.contour(ARvals, Svals, mO3vals, levels=levels, linewidths=0.5)
    plt.clabel(cs, colors="black", fontsize=7.0)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("mO3 (g/s^3)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)
    
    # Payload mass
    fig2   = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(mpayvals[mpayvals > 0]), np.max(mpayvals), 21)
    cs     = plt.contour(ARvals, Svals, mpayvals, levels=levels, linewidths=0.5)
    plt.clabel(cs, colors="black", fontsize=8.0)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Payload mass (g)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)    

    # Turn rate
    fig3   = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(Omegavals[Omegavals > 0]), np.max(Omegavals), 21)
    cs     = plt.contour(ARvals, Svals, Omegavals, levels=levels, linewidths=0.5)
    plt.clabel(cs, colors="black", fontsize=8.0)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Turn rate (rad/s)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)    
    
    # Turn radius
    fig4   = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(Rvals[Rvals > 0]), np.max(Rvals), 21)
    cs     = plt.contour(ARvals, Svals, Rvals, levels=levels, linewidths=0.5)
    plt.clabel(cs, colors="black", fontsize=8.0)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Turn radius (m)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)       
    
    # Wing tip deflection
    fig5 = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(dbvals[dbvals > 0]), np.max(dbvals), 11)
    cs = plt.contour(ARvals, Svals, dbvals, levels=levels, linewidths=0.5)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.clabel(cs, colors="black", fontsize=8.0)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Wing Tip Deflection (-)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)
    
    # Lift coefficient
    fig6 = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(CLvals[CLvals > 0]), np.max(CLvals), 11)
    cs = plt.contour(ARvals, Svals, CLvals, levels=levels, linewidths=0.5)
    plt.clabel(cs, colors="black")
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Lift coefficient (-)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)
    
    # Thrust
    fig7 = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(Tvals[Tvals > 0]), np.max(Tvals), 11)
    cs = plt.contour(ARvals, Svals, Tvals, levels=levels, linewidths=0.5)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.clabel(cs, colors="black", fontsize=7.0)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Thrust (N)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92) 
    
    # Maximum thrust
    fig8 = plt.figure(figsize=(4, 3.5), dpi=150)
    # Levels here are the same as for Tvals, so that thrust contours can be
    # compared more easily
    levels = np.linspace(np.min(Tvals[Tvals > 0]), np.max(Tvals), 11)
    cs = plt.contour(ARvals, Svals, Tmaxvals, levels=levels, linewidths=0.5)
    plt.plot(ARopt, Sopt, marker=marker, color="red", markersize=20)
    plt.clabel(cs, colors="black", fontsize=7.0)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Maximum Thrust (N)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)
    
    """
    # Load factor
    fig9 = plt.figure(figsize=(4, 3.5), dpi=150)
    levels = np.linspace(np.min(Nvals[Nvals > 0]), np.max(Nvals), 11)
    cs = plt.contour(ARvals, Svals, Nvals, levels=levels, linewidths=0.5)
    plt.clabel(cs, colors="black", fontsize=8.0)
    plt.grid()
    plt.xlabel("Aspect ratio (-)")
    plt.ylabel("Wing area ($m^2$)")
    plt.title("Load Factor (-)")
    plt.subplots_adjust(left=0.18, right=0.98, bottom=0.13, top=0.92)
    """
    
    plt.show(block=True)
    
