import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np


def angle(displace):
    I_yy=6.1902*pow(10, -6)
    F=6.13 #1/4 of drone weight * g. the drone weight is approximately 2500 gms
    r=2*displace
    theta_rad= ((-1*F)*(2*displace))/I_yy
    theta_deg=(theta_rad*180)/(math.pi)
    #print("\ndisplacement:",displace,"m Roll angle:",theta_deg,"degrees")
    return theta_deg


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #displace_cm = int(input("INPUT NOW\nplease enter displacement(cm): "))
    displace_mm = list(range(0, 50))
    #print(displace_mm)

    lst_disp = [None] * 50
    lst_angle = [None] * 50

    for i in displace_mm:
        lst_disp[i] = i/1000
        lst_angle[i]=((angle(i/1000))/360)%360
        print("\nDisplacement:",lst_disp[i],"m Tilt:",lst_angle[i],"degrees")
