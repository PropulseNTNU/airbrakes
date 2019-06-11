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
path = "Penumbra/rockets/sleipner_analytic/"
file = "init_rocket.r"

# Enviorment / rocket initzialisation
sleipner = Rocket1.RocketSimple.from_file(file, path)
sleipner.setMass(27235e-3)
sleipner.setCOM(-1340e-3)
initialInclination = 6 * np.pi / 180
rampLength = 5.2
simTime = 30
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
position, linearVelocity = shooter.simpleShoot(6, 1, 3048, 5)
# Params (initial time of deplyment,
# change in time as optimization happens,
# ideal apogee,
# tollerance)


lookUpTable=[]
hoydeN=0;
hoyde=0;
diff=0;
for i in range(len(position)):
    hoydeN = int(np.floor(position[i]))
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
print(max(position))
print(len(lookUpTable))

plt.plot(lookUpTable)
plt.ylabel('Velocity graph')
plt.show()

plt.plot(position)
plt.ylabel('Height graph')
plt.show()


"""
    Change simple shoot to this when running this code! 

    def simpleShoot(self, t0, dt, targetApogee, tol):
        t = t0
        # Loop
        apogee = targetApogee + 2 * tol
        position = None
        linearVelocity = None
        while(np.abs(targetApogee - apogee) > tol):
            trajectory = traBrakes.calculateTrajectoryWithAirbrakes(self.rocket,\
                self.initialInclination, self.launchRampLength, self.timeStep,\
                self.simulationTime, t, self.Cbrakes_in, self.dragDev,\
                self.windObj)
            position = -trajectory[1][:,2]
            linearVelocity = - trajectory[4][:,2]

            apogee = np.max(-trajectory[1][:,2])
            a = np.sign(targetApogee - apogee)
            if t != t0:
                if a != b: dt /= 2
            else:
                b = a
            t += a * dt
            print(str(t).ljust(20, ' ') + str(apogee).ljust(20, ' '))
        return position, linearVelocity
"""