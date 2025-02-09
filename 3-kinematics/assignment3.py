#%%
import vedo as vd
vd.settings.default_backend= 'vtk'
import numpy as np
from vedo.pyplot import plot

#%% class for a robot arm
def Rot(angle, axis):
    # calculate the rotation matrix for a given angle and axis using Rodrigues' formula
    # return a 3x3 numpy array
    # also see scipy.spatial.transform.Rotation.from_rotvec
    axis = np.array(axis)
    axis = axis/np.linalg.norm(axis)
    I = np.eye(3)
    K = np.array([[0, -axis[2], axis[1]],
                    [axis[2], 0, -axis[0]],
                    [-axis[1], axis[0], 0]])
    R = I + np.sin(angle)*K + (1-np.cos(angle))*np.dot(K,K)
    return R


def UpdatePlot(iterations, gradient_norms): 
    global fig
    global method

    if method == 'grad_desc':
        graph_title = "Gradient Descent"
    else:
        graph_title = "Gauss Newton"

    ################# plot
    if fig is not None:
        plt.at(1).remove(fig)

    fig = plot(
        iterations, gradient_norms,
        title= graph_title,
        xtitle="steps",
        ytitle="gradient value",
        ).clone2d(pos='center', size=1)

    plt.at(1).add(fig)


class SimpleArm:
    def __init__(self, n=3, link_lengths=None):
        # a simple arm with n unit length links connected by hinge joints
        # The base is assumed to be at the origin
        # The joints are assumed to be at the end of each link, with the first joint at the base and the last joint at the end effector
        # even though the end effector is not a joint
        # The axis of rotation for each joint are assumed to be the z-axis
        # The arm is initialized to lie along the x-axis
        
        self.n = n # number of links
        self.angles = [0]*self.n # joint angles, starting from the base joint to the end effector
        if link_lengths is None:
            self.link_lengths = [1]*self.n
        else:
            self.link_lengths = link_lengths
        # self.Jl is a matrix that contains joints position in local coordinates.
        # Each row contains the coordinates of a joint
        # Number of joints is n+1, including the base and end effector
        self.Jl = np.zeros((self.n+1, 3)) 
        for i in range(1,n+1): # we start from 1 because the base joint is at the origin (0,0,0) and finish the end effector is at the end of the last link
            self.Jl[i,:] = np.array([self.link_lengths[i-1], 0, 0]) # initialize joint positions to lie along the x-axis

        self.Jw = np.zeros((self.n+1, 3)) # joint positions in world coordinates
        self.Rw = [np.eye(3) for _ in range(self.n+1)]
        self.FK()
    
    
    
    ###############################################
    #                                             #
    #           FK (Forward Kinematics)           #
    #                                             #
    ###############################################   

    def FK(self, angles=None): 
        # calculate the forward kinematics of the arm

        # angles is a list of joint angles. If angles is None, the current joint angles are used
        if angles is not None:
            self.angles = angles
        
        # Reset world positions
        self.Jw = np.zeros((self.n+1, 3))
        self.Rw[0] = np.eye(3)

        # Initial rotation matrix
        Ri = np.eye(3)

        for i in range(1,self.n+1):
            R_joint = Rot(self.angles[i-1], [0, 0, 1]) 
            Ri = Ri @ R_joint
            self.Rw[i] = Ri
            self.Jw[i, :] = Ri @ self.Jl[i, :] + self.Jw[i-1, :] 
        return self.Jw[-1,:]

    ###############################################
    #                                             #
    #           VELOCITY JACOBIAN                 #
    #                                             #
    ###############################################
    def velocity_jacobian(self, angles=None):
        """
        Calculate the velocity jacobian of the arm
        input: joint angles θ = [θ₁, θ₂, ..., θₙ]
        output: Jacobian matrices J_theta  
        """
        if angles is not None:
            self.angles = angles
        else:
            self.FK()

        end_effector_pos = self.Jw[-1, :]
        J_theta = np.zeros((3, self.n))  # Jacobian for angular velocities
        
        local_z  = np.array([0, 0, 1],dtype=float)  # w (omega) = Rotation axis for all joints
        
        for i in range(self.n):
            R_i = np.eye(3)

            for k in range(i):
                R_i = R_i @ Rot(self.angles[k], [0, 0, 1])

            z_i = R_i @ local_z 

            joint_pos_i = self.Jw[i, :]
            r = end_effector_pos - joint_pos_i            

            # Angular velocity component (cross product with omega)
            J_theta[:, i] = np.cross(z_i, r)
            
        return J_theta
    
    ###############################################
    #                                             #
    #           IK (Inverse Kinematics)           #
    #                                             #
    ###############################################     
      
    def IK(self, target, method=None, max_iter=None, alpha=None):
        target = np.array(target, dtype=float)
        
        # Initialize lists to store iterations and gradient norms
        iterations = []
        gradient_norms = []
        
        for i in range(max_iter):
            # Forward kinematics to get current end effector
            current_pos = self.FK()
            error = target - current_pos   

            # If close enough, stop
            if np.linalg.norm(error) < 1e-3:
                break

            # Compute Jacobian
            J = self.velocity_jacobian()
            jacobian_norm = np.linalg.norm(error)  # Calculate Jacobian norm
 
            # Track iteration and Jacobian norm
            iterations.append(i)
            gradient_norms.append(jacobian_norm)

            if method == 'grad_desc':
                # Jacobian transpose method
                # dTheta = alpha * J^T * error
                dTheta = alpha * (J.T @ error)

            elif method == 'gauss_newton':
                # Gauss-Newton method uses pseudoinverse (J^T J)^{-1} J^T
                # dTheta = (J^+)* error
                # Pseudoinverse for a 3xN Jacobian
                J_pinv = np.linalg.pinv(J)   # robust pseudo-inverse
                dTheta = J_pinv @ error

            # Update angles
            self.angles += dTheta
        
        # Call the UpdatePlot function after the loop
        UpdatePlot(iterations, gradient_norms)

        return self.angles
    

    def draw(self):
        vd_arm = vd.Assembly()
        vd_arm += vd.Sphere(pos = self.Jw[0,:], r=0.05)
        for i in range(1,self.n+1):
            vd_arm += vd.Cylinder(pos = [self.Jw[i-1,:], self.Jw[i,:]], r=0.02)
            vd_arm += vd.Sphere(pos = self.Jw[i,:], r=0.05)
        return vd_arm
    

    def visualizeJacobian(self, J):
        global activeJacobian  # Use the active joint index selected by the user

        # Starting point of the arrow is the position of the selected joint
        arrow_start = self.Jw[-1, :]

        # Direction of the arrow is the column of the Jacobian corresponding to the joint
        velocity_vector = J[:, activeJacobian]
        arrow_stop = arrow_start + velocity_vector

        # Create and return a single arrow in 3D
        arrow = vd.Arrow(arrow_start, arrow_stop, s=0.01,c='blue', alpha=0.4)  # Arrow in 3D

        return arrow

        
        

#%%
activeJoint = 0
activeJacobian = 0
IK_target = [1,1,0]
fig = None
method = 'grad_desc'

def OnSliderAngle(widget, event):
    global activeJoint
    arm.angles[activeJoint] = widget.value
    arm.FK()

    # Compute the Jacobian
    J = arm.velocity_jacobian()

    # Remove the previous Jacobian arrow if it exists
    plt.at(0).remove("JacobianArrow")

    # Visualize the new Jacobian arrow
    arrow = arm.visualizeJacobian(J)
    arrow.name = "JacobianArrow"  # Assign a name for easier removal
    plt.at(0).add(arrow)

    # Update arm drawing
    plt.at(0).remove("Assembly")
    plt.at(0).add(arm.draw())
    plt.at(0).render()

def OnCurrentJoint(widget, event):
    global activeJoint
    activeJoint = round(widget.value)
    sliderAngle.value = arm.angles[activeJoint]

def OnCurrentJacobian(widget, event):
    global activeJacobian
    activeJacobian = round(widget.value)
    print("Active Jacobian: ", activeJacobian)
 

def LeftButtonPress(evt): 
    if plt.renderers.index(plt.renderer) != 0:
        return 
    global IK_target
    global method
    IK_target = evt.picked3d
    plt.at(0).remove("Sphere")
    plt.at(0).remove("JacobianArrow")
    plt.at(0).add(vd.Sphere(pos = IK_target, r=0.05, c='b'))
    plt.at(0).render()

    arm.IK(IK_target, method, max_iter=1000, alpha=0.01)
    plt.at(0).remove("Assembly")
    plt.at(0).add(arm.draw())
    plt.at(0).render()



def IK_method(obj, ename):
    if plt.renderers.index(plt.renderer) != 1:
        return 
    global method
    if method == 'grad_desc':
        method = 'gauss_newton'
    else:
        method = 'grad_desc'
    state = method_btn.switch()  # Switch the button text
     

arm = SimpleArm(n=4,link_lengths=[1.4, 0.5, 1.0,0.5])
plt = vd.Plotter(N=2)
plt.at(0).add(arm.draw())
plt.at(0).add(vd.Sphere(pos=IK_target, r=0.05, c='b').draggable(True))  
plt.at(0).add(vd.Plane(s=[2.1*arm.n,2.1*arm.n]))


sliderCurrentJoint = plt.at(0).add_slider(OnCurrentJoint, 0, arm.n-1, 0, title="Current joint",title_size=0.8, pos=[(0.05, 0.06), (0.20, 0.06)], delayed=True)
sliderAngle =  plt.at(0).add_slider(OnSliderAngle,-np.pi,np.pi,0., title="Joint Angle",title_size=0.8, pos=[(0.30, 0.06), (0.45, 0.06)])
sliderCurrentJacobian = plt.at(0).add_slider(OnCurrentJacobian, 0, arm.n-1, 0, title="Joint Arrow",title_size=0.8, pos=[(0.05, 0.92), (0.20, 0.92)], delayed=True)
method_btn = plt.at(1).add_button(
                fnc=IK_method,               
                states=["click to Gauss-Newton", "click to Gradient"],
                c=["w", "w"],     
                bc=["dg", "dv"],        
                pos=(0.5, 0.14),
                size=20,      
                font="courier",
                bold=True,
                italic=False,
            )
            
plt.at(0).add_callback('LeftButtonPress', LeftButtonPress) # add Mouse callback


plt.user_mode('2d')
plt.show(zoom="tightest" ,interactive=True)
plt.close()

# %%