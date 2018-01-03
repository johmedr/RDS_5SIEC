from pinocchio.utils import *

class FootSteps:
    '''
    The class stores three functions of time: left, right and flyingFoot.
    Each function is piecewise constant. For each function, the user can ask
    what is the value of this function at time t.

    The storage is composed of three lists for left, right and flyingFoot, and a list for time.
    The list of times stores the time intervals, i.e. each element of the list is
    the start of a time interval. The first element of the list is 0.
    The value of the functions left,right,flyingFoot one this time interval is stored at
    the same position is their respective list (i.e. value of left on interval
    [ time[i],time[i+1] ] is stored in left[i].

    The 4 lists are set up using function addPhase().
    The values of functions left,right,flyingFoot can be accessed through the function
    getPhaseType(t), getLeftPosition(t), getRightPosition(t).
    PhaseType are 'left' (meaning left foot is flying, right foot is fixed), 'right' (ie the opposite)
    or 'none' (meaning no foot is flying, both are fixed on the ground).

    Additionnally, functions getLeftNextPosition(t),
    getRightNextPosition(t) can be used to get the next position of the
    flying foot (in that case, additional work is needed to compute the
    position of flying foot at time t by interpolating getLeftPosition(t)
    and getLeftNextPosition(t).

    Functions getPhaseStart(t), getPhaseDuration(t) and getPhaseRemaining(t)
    can be used to get the starting time, the duration and the remaining time of the
    current phase at time t.
    '''

    def __init__( self, right,left ):
        '''
        The class is initiated from the initial positions of left and right feet.
        '''

        self.right = [ right, ]
        self.left  = [ left,  ]
        self.time  = [ 0.,    ]
        self.flyingFoot = []

    def addPhase( self, duration, foot, position = None ):
        '''
        Add a phase lasting <duration> where the flyhing foot <foot> (either 'left' or 'right')
        moves to <position> (being a vector or a SE3 placement).
        Alternatively, <foot> might be set to 'none' (i.e double support). In that case, <position>
        is not specified (or is set to None, default).
        '''

        assert( foot == 'left' or foot == 'right' or foot == 'none' )
        self.time.append( self.time[-1]+duration )
        self.right.append( self.right[-1] )
        self.left.append( self.left[-1] )
        self.flyingFoot.append(foot)
        if foot=='left':
            self.left[-1] = position
        elif foot=='right':
            self.right[-1] = position

    def getIndexFromTime(self, t):
        '''Return the index i of the interval containing t, i.e. t in time[i],time[i+1] '''
        if t>self.time[-1]: return len(self.time)-1
        return next( i for i,ti in enumerate(self.time) if ti>t ) - 1

    def getPhaseType(self, t ):
        i = self.getIndexFromTime(t)
        return self.flyingFoot[i]

    def isDoubleFromLeftToRight(self, t ):
        '''
        Suppose that phase at time <t> is a double support phase.
        Return True if the previous phase is left and/or the next phase is right.
        '''
        assert(self.getPhaseType(t)=='none')
        i = self.getIndexFromTime(t)
        if i>0:
            return self.flyingFoot[i-1]=='left'
        else:
            return self.flyingFoot[i+1]=='right'

    def getLeftPosition(self,t):
        i = self.getIndexFromTime(t)
        return self.left[i]

    def getRightPosition(self,t):
        i = self.getIndexFromTime(t)
        return self.right[i]

    def getLeftNextPosition(self,t):
        i = self.getIndexFromTime(t)
        i = i+1 if i+1< len(self.time) else i
        return self.left[i]

    def getRightNextPosition(self,t):
        i = self.getIndexFromTime(t)
        i = i+1 if i+1< len(self.time) else i
        return self.right[i]

    def getPhaseStart(self,t):
        i = self.getIndexFromTime(t)
        return self.time[i]

    def getPhaseDuration(self,t):
        i = self.getIndexFromTime(t)
        return self.time[i+1]-self.time[i]
        
    def getPhaseRemaining(self,t):
        i = self.getIndexFromTime(t)
        return self.time[i+1]-t


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