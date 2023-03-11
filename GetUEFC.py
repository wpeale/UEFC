# Implement UEFC (Unified Engineering Flight Competition) aircraft as a class.

import numpy as np

from GetWingDimensions import GetWingDimensions

from GetWfuse      import GetWfuse
from GetWingWeight import GetWingWeight
from GetWpay       import GetWpay 
from GetWeight     import GetWeight
from GetMass       import GetMass

from GetV       import GetV
from GetCL      import GetCL
from Getspaneff import Getspaneff
from GetCDpay   import GetCDpay
from GetCD      import GetCD

from Getepsilon import Getepsilon
from Getdb      import Getdb

from GetRequiredThrust import GetRequiredThrust
from GetMaxThrust      import GetMaxThrust
from GetExcessThrust   import GetExcessThrust

from GetOmega import GetOmega

class UEFC:
    
    def __init__(self):
        
        # EXCEPT FOR CHANGING THE CONSTANTS AS YOU LOOK AT DIFFERENT
        # TAPER, DBMAX, ETC YOU SHOULD NOT NEED TO CHANGE THIS CLASS.
        
        # Geometry parameters
        self.taper    = 0.5   # taper ratio
        self.dihedral = 10.0  # Wing dihedral (degrees)
        self.tau      = 0.12  # thickness-to-chord ratio
        
        # Aerodynamic parameters
        self.CLdes = 0.85  # maximum CL wing will be designed to fly at (in cruise)
        self.e0    = 1.0   # Span efficiency for straight level flight
        
        # Wing bending and material properties
        self.dbmax   = 0.1     # tip displacement bending constraint
        self.rhofoam = 25.5    # kg/m^3. Dow Blue foam (low load)
        self.Efoam   = 12.0E6  # Pa.     Dow Blue foam (low load)
        
        # Other modeling parameters
        self.rho = 1.225     # air density kg/m^3
        self.mu  = 1.789E-5  # dynamic viscosity of air N s/m^2
        self.g   = 9.81      # gravity, m/s^2
    
    # opt_vars is a vector representing the optimization variables
    # opt_vars[0]: Load factor (-)
    # opt_vars[1]: Turn radius (meters)
    # opt_vars[2]: Payload mass (grams)
    
    # YOU SHOULD NOT NEED TO CHANGE THESE METHOD CALLS
    def fuselage_weight(self, AR, S):  
        return GetWfuse(self, AR, S)  # Fuselage weight (N)
    
    def wing_weight(self, AR, S):
        return GetWingWeight(self, AR, S)  # Wing weight (N)
    
    def payload_weight(self, opt_vars, AR, S):
        return GetWpay(self, opt_vars, AR, S)  # Payload weight (N)
    
    def weight(self, opt_vars, AR, S):
        return GetWeight(self, opt_vars, AR, S)  # Total weight; breakdown (N)
    
    def mass(self, opt_vars, AR, S):
        return GetMass(self, opt_vars, AR, S)  # Total mass, and a breakdown (g)
    
    def flight_velocity(self, opt_vars, AR, S):
        return GetV(self, opt_vars, AR, S)  # Flight velocity (m/s)
      
    def lift_coefficient(self, opt_vars, AR, S):
        return GetCL(self, opt_vars, AR, S)  # Lift coefficient (-)
    
    def span_efficiency(self, opt_vars, AR, S):
        return Getspaneff(self, opt_vars, AR, S)  # Wing span efficiency (-)
    
    def payload_drag_coefficient(self, opt_vars, AR, S):
        return GetCDpay(self, opt_vars, AR, S)  # Payload drag coefficient (-)
    
    def drag_coefficient(self, opt_vars, AR, S):
        return GetCD(self, opt_vars, AR, S)  # Total drag coefficient; breakdown
    
    def max_camber(self):
        return Getepsilon(self)  # Maximum wing camber (-)
    
    def wing_tip_deflection(self, opt_vars, AR, S):
        return Getdb(self, opt_vars, AR, S)  # Wing tip deflection / wingspan
    
    def required_thrust(self, opt_vars, AR, S):
        return GetRequiredThrust(self, opt_vars, AR, S)  # Required thrust (N)
    
    def maximum_thrust(self, V):
        return GetMaxThrust(self, V)  # Maximum thrust (N)
    
    def excess_thrust(self, opt_vars, AR, S):  # Maximum - required thrust (N)
        return GetExcessThrust(self, opt_vars, AR, S)
    
    def wing_dimensions(self, AR, S):
        return GetWingDimensions(self, AR, S)
    
    def turn_rate(self, opt_vars, AR, S):  # Turn rate (rad/s)
        return GetOmega(self, opt_vars, AR, S)


if __name__ == "__main__":
    
    pass
    
    """
    # Simple test case. Feel free to modify this part of the file.
    aircraft = UEFC()
    
    AR = 10.1
    S  = 0.228  # m^2
    
    opt_vars = np.array([1.2, 10.0, 300])  # N, R, mpay
    
    mass_data   = aircraft.mass(opt_vars, AR, S)
    weight_data = aircraft.weight(opt_vars, AR, S)
    
    print()
    for element, value in mass_data["Breakdown"].items():
        print(element + " mass: \t%0.1f g" % value)
        
    print("Total mass:    \t%0.1f g" % mass_data["Total"])    
        
    print()
    for element, value in weight_data["Breakdown"].items():
        print(element + " weight:    \t%0.4f N" % value)
        
    print("Total weight:    \t%0.4f N" % weight_data["Total"])
    
    V       = aircraft.flight_velocity(opt_vars, AR, S)
    CL      = aircraft.lift_coefficient(opt_vars, AR, S)
    e       = aircraft.span_efficiency(opt_vars, AR, S)
    CDpay   = aircraft.payload_drag_coefficient(opt_vars, AR, S)
    CD_data = aircraft.drag_coefficient(opt_vars, AR, S)
    
    print()
    print("Flight velocity:  \t%0.4f m/s" % V)
    print("Lift coefficient: \t%0.4f"     % CL)
    print("Span efficiency:  \t%0.4f"     % e)
    
    print()
    for element, value in CD_data["Breakdown"].items():
        print(element + " drag coefficient:    \t%0.4f" % value) 
    
    print("Total drag coefficient:    \t%0.4f" % CD_data["Total"])
    
    epsilon = aircraft.max_camber()
    db      = aircraft.wing_tip_deflection(opt_vars, AR, S)
    
    print()
    print("Wing max camber:                \t%0.4f" % epsilon)
    print("Wing tip deflection / wingspan: \t%0.4f" % db)
    
    T_req    = aircraft.required_thrust(opt_vars, AR, S)
    T_max    = aircraft.maximum_thrust(V)
    T_excess = aircraft.excess_thrust(opt_vars, AR, S)
    
    print()
    print("Required thrust: \t%0.4f N" % T_req)
    print("Maximum thrust:  \t%0.4f N" % T_max)
    print("Excess thrust:   \t%0.4f N" % T_excess)
    
    N     = opt_vars[0]
    R     = opt_vars[1]
    mpay  = mass_data["Breakdown"]["Payload"]
    Omega = aircraft.turn_rate(opt_vars, AR, S) 
    
    print()
    print("Load factor:  \t%0.3f"       % N)
    print("Turn radius:  \t%0.2f m"     % R)
    print("Payload mass: \t%0.0f g"     % mpay)
    print("Turn rate:    \t%0.2f rad/s" % Omega)
    print("mpay Omega:   \t%0.0f g/s"   % (mpay*Omega))
    """
    
    
