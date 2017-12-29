import numpy as np

"""
Constraint knees rotation > 0
"""
DEFAULT_CONSTRAINTS = (
        {'type': 'ineq', 
          'fun': lambda x: np.array(x[6])}, 
        {'type': 'ineq', 
          'fun': lambda x: np.array(x[13])}
        )

"""
Can only rotate around y
"""
MOVE_PLANE_XZ_CONSTRAINTS = (
        {'type': 'eq', 
          'fun': lambda x: np.array(x[3])}, 
        {'type': 'eq', 
          'fun': lambda x: np.array(x[5])},
        {'type': 'eq', 
          'fun': lambda x: np.array(x[7])}, 
        {'type': 'eq', 
          'fun': lambda x: np.array(x[9])},

        {'type': 'eq', 
          'fun': lambda x: np.array(x[10])},
        {'type': 'eq', 
          'fun': lambda x: np.array(x[12])},
        {'type': 'eq', 
          'fun': lambda x: np.array(x[14])},
        {'type': 'eq', 
          'fun': lambda x: np.array(x[16])},
        )


"""
Symetric move
"""
SYMETRIC_CONSTRAINTS = tuple(
        [{'type': 'eq', 
          'fun': lambda x: np.array(x[i] - x[i + 7])} 
        for i in range(3, 9)]
        )

"""
Torso translation Z
"""
TORSO_CONSTRAINTS_TZ = (
        {'type': 'eq', 
          'fun': lambda x: np.array(x[0])}, 
        {'type': 'eq', 
          'fun': lambda x: np.array(x[1])}
          )