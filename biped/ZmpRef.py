import numpy as np
from FootSteps import FootSteps

import matplotlib.pyplot as plt

class ZmpRef (object):
    def __init__ (self, footsteps) :
        assert(isinstance(footsteps, FootSteps))
        self.footsteps = footsteps

        self.steps = []


    # Operator ()
    def __call__ (self, t):
        assert(t >= 0 and t < self.footsteps.time[-1])

        phase = self.footsteps.getPhaseType(t)

        if phase == 'left':
            zmp_pos = np.array(self.footsteps.getRightPosition(t))

        elif phase == 'right':
            zmp_pos = np.array(self.footsteps.getLeftPosition(t))

        elif phase == 'none': 
            
            delta_t = self.footsteps.getPhaseDuration(t)
            t0 = self.footsteps.getPhaseStart(t)

            i = self.footsteps.getIndexFromTime(t)

            right = np.array(self.footsteps.getRightPosition(t))
            left = np.array(self.footsteps.getLeftPosition(t))

            if self.footsteps.isDoubleFromLeftToRight(t):
                start = right
                target = left
            else: 
                start = left
                target = right

            if i == 0: 
                start = 0.5 * (np.array(self.footsteps.getLeftPosition(t)) + np.array(self.footsteps.getRightPosition(t)))
            elif i == len(self.footsteps.flyingFoot) - 1:
                target =  0.5 * (np.array(self.footsteps.getLeftPosition(t)) + np.array(self.footsteps.getRightPosition(t)))


            zmp_pos = (target - start) / delta_t * (t - t0) + start


        return zmp_pos

if __name__ == "__main__":
    # Define 6 steps forward, starting with the left foot and stoping at the same forward position.
    footsteps = FootSteps( [0.0,-0.1] , [0.0,0.1] )
    footsteps.addPhase( .3, 'none' )
    footsteps.addPhase( .7, 'left' , [0.1,+0.1] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'right', [0.2,-0.1] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'left' , [0.3,+0.1] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'right', [0.4,-0.1] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'left' , [0.5,+0.1] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'right', [0.5,-0.1] )
    footsteps.addPhase( .5, 'none' )



    zmp = ZmpRef(footsteps)

    t = np.arange(0., footsteps.time[-1] - footsteps.time[-1] / 2000., footsteps.time[-1] / 2000.)
    print t

    # print [footsteps.getLeftPosition(ti) for ti in t]

    plt.plot(t, [zmp(ti)[0] for ti in t], 'b', t, [zmp(ti)[1] for ti in t], 'g')
    plt.show()
