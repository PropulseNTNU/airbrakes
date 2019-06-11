import sys
sys.path.append("Penumbra/Rocket")
sys.path.append("Penumbra/Trajectory")
sys.path.append("Penumbra/Optimization")
sys.path.append("Penumbra/Forces")

import Trajectory
import TrajectoryWithBrakes
import Rocket1
import Optimization as optim
import numpy as np
import matplotlib.pyplot as plt

# Correct rocket
path = "Penumbra/rockets/test_rocket/"
file = "init_rocket.r"

# Enviorment / rocket initzialisation
sleipner = Rocket1.RocketSimple.from_file(file, path)
sleipner.setMass(800e-3)
sleipner.setCOM(-500e-3)

initialInclination = 6 * np.pi / 180
rampLength = 1.5
simTime = 20
timeStep = 0.03

# Tbrakes - deployment time for airbrakes
# Cbrakes_in - Cbrakes coefficent (ask Ben)
rho = 1.225 
C_d = 1.28
A_max = 0.006636
C_Max = (1/2)*rho*C_d*A_max
Cbrakes_in = 0.9 * C_Max
print("CBrakes: ", Cbrakes_in)

shooter = optim.ShootingOptimz(sleipner, initialInclination, rampLength, timeStep, simTime, Cbrakes_in)
# Params (rocket, ..., Cbrakes_in)
position, linearVelocity = shooter.simpleShoot(10, 1, 400, 5)
# Params (initial time of deplyment,
# change in time as optimization happens,
# ideal apogee,
# tollerance)



lookUpTable=[]
hoydeN=0;
hoyde=0;
diff=0;
for i in range(len(position)):
    hoydeN= int(np.floor(position[i]))
    if hoydeN < hoyde:
        continue
    if hoydeN>hoyde:
        #var2=int(np.floor(position[:,2][i+1]))
        diff=hoydeN-hoyde
        if i==0:
            a=(linearVelocity[i])/diff
            for j in range(diff):
                lookUpTable.append(a*j)
                hoyde+=1
        else:
            a=(linearVelocity[i]-(linearVelocity[i-1]))/diff
            for j in range(diff):
                lookUpTable.append(a*j+lookUpTable[hoyde-j-1])
                hoyde+=1
    lookUpTable.append(linearVelocity[i])
    hoyde+=1
print(lookUpTable)

print(len(lookUpTable))
print(max(lookUpTable))

plt.plot(lookUpTable)
plt.ylabel('Velocity graph')
plt.show()

plt.plot(position)
plt.ylabel('Velocity graph')
plt.show()