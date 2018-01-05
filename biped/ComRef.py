from FootSteps import FootSteps
from FactorGraph import Factor, FactorGraph
from ZmpRef import ZmpRef

import numpy as np
import numpy.linalg as npl
from pinocchio.utils import *

import matplotlib.pyplot as plt

class ComRef: 
    def __init__(self, zmpRef, nbPoints, cz):
        assert(isinstance(zmpRef, ZmpRef))
        
        self.zmpRef = zmpRef
        
        self.delta_t = self.zmpRef.get_trajectory_duration() / float(nbPoints)
        
        N = 8 


        M = eye(1)

        g = 9.81
        omega_p = cz / g

        x_ref = np.zeros((N, nbPoints)) 
        zmpRef_t = self.zmpRef((nbPoints - 1) * self.delta_t)
        x_ref[0, 0] = zmpRef_t[0]
        x_ref[1, 0] = zmpRef_t[1]

        # Swap time variation
        for t_i in range(nbPoints - 1, 0, -1): 
            self.factor_graph = FactorGraph(1, N)
            print x_ref

            for n in range(N - 2): 
                self.factor_graph.addFactorConstraint(
                    [Factor(n, M)], 
                    M * (x_ref[n, t_i] + self.delta_t * x_ref[n+2, t_i])
                        )

                # print M * (x_ref[n, t_i] + self.delta_t * x_ref[n+2, t_i])

            zmpRef_t = self.zmpRef(t_i * self.delta_t)

            self.factor_graph.addFactor(
                    [Factor(0, M),
                     Factor(4, - omega_p * M)],
                     zmpRef_t[0]
                     )

            self.factor_graph.addFactor(
                    [Factor(1, M),
                     Factor(5, - omega_p * M)], 
                     zmpRef_t[1]
                     )

            # self.factor_graph.show()

            for n in range(N): 
                x_ref[n, t_i + 1] =  self.factor_graph.solve()[n]



        self.comRef = x_ref



        # for n in range(N):
        #   self.factor_graph.addFactorConstraint([
        #               Factor(n , M), 
        #               Factor(n + 2, self.delta_t_i * M), 
        #               Factor(n + (t_i + 1) * N, -M)
        #               ])

            # zmpRef_t = self.zmpRef(t_i * self.delta_t)

            # self.factor_graph.addFactor([
            #                     Factor(0 + t_i * N, M), 
            #                     Factor(4 + t_i * N, - omega_p * M) 
            #                     ], M * zmpRef_t[0]
            #                     )
            # self.factor_graph.addFactor([
            #                     Factor(1 + t_i * N, M), 
            #                     Factor(5 + t_i * N, - omega_p * M) 
            #                     ], M * zmpRef_t[1]
            #                     )

        # zmpRef_t = np.array([self.zmpRef(t_i * self.delta_t) for t_i in range(nx)])

        # self.factor_graph.addFactor([
        #                         Factor(0, Mt), 
        #                         Factor(4, - omega_p * Mt) 
        #                         ], zmpRef_t[:,0]
        #                         )
        # self.factor_graph.addFactor([
        #                         Factor(1, Mt), 
        #                         Factor(5, - omega_p * Mt) 
        #                         ], zmpRef_t[:,1]
        #                         )

        # self.comRef = self.factor_graph.solve() 



if __name__ == "__main__":
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

    zmpRef = ZmpRef(footsteps)

    comRef = ComRef(zmpRef, 100, 1)

    plt.plot([t * comRef.delta_t for t in range(100)], comRef.comRef[0, :])
    plt.show()
    # print comRef.comRef

