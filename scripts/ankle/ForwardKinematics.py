import numpy as np
import os
import ankle

class forwardKinematics2:
    def __init__(self, weights):
        """
        Sets up the ANN. 
        
        NOTE: this function will need to be adjusted if more layers are added to the ANN.
        """
        self.w1 = weights[0]
        self.b1 = weights[1]

        self.w2 = weights[2]
        self.b2 = weights[3]
    
    def predict(self, x):
        """ 
        Used the weights and biases to run a forward pass of the ANN to predict the ankle pitch and roll.
        Inputs:
            - x should be an array and have shape (1,2)--> [[left motor position, right motor position]]
        Outputs:
            - [[ankle pitch, ankle roll]] 
        """
        l1 = (np.maximum(0, np.dot(x, self.w1) + self.b1))
        return np.dot(l1 ,self.w2) + self.b2

    def print_weights(self):
        print('w1:')
        print('   ',end ='')
        print(self.w1)
        print('b1:')
        print('   ',end ='')
        print(self.b1)
        print('w2:')
        print('   ',end ='')
        print(self.w1)
        print('b2:')
        print('   ',end ='')
        print(self.b1)

module_dir = os.path.dirname(ankle.__file__)
weights = np.load(os.path.join(module_dir, 'FKWeights.npz'), allow_pickle=True)
forward_kinematics = forwardKinematics2(weights['arr_0'])

# Given current motor positions, find roll and pitch of the food

# Uses rpy convention
def fk(inner_pos, outer_pos):

    foot_pitch, foot_roll = forward_kinematics.predict([inner_pos, outer_pos])
    return [foot_roll, foot_pitch]