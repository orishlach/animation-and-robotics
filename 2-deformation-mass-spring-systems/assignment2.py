#%%
import vedo as vd
vd.settings.default_backend= 'vtk'
from vedo.pyplot import plot

from vedo import show
import numpy as np

from abc import ABC, abstractmethod
import numdifftools as nd
from scipy.sparse import coo_matrix
import triangle as tr # pip install triangle    

usage_txt = (
            "Left click on vertice :rightarrow Pick a point\n"
            "Right click and drag :rightarrow update last point to new postion\n"
            "A :rightarrow Run Gradient Descent with ZeroLengthSpringEnergy\n"
            "B :rightarrow Run Newton's Method with ZeroLengthSpringEnergy\n"
            "C :rightarrow Run Gradient Descent with SpringEnergy\n"
            "D :rightarrow Run Newton's Method with SpringEnergy\n"
            "R :rightarrow Reset mesh\n"
        )

usage = vd.Text2D(usage_txt, font='Calco', pos="top-left", s=0.7, bg='r9', alpha=0.25)

msg = vd.Text2D(font='Calco', pos="bottom-left", s=0.8, bg='r9', alpha=0.25)

#%% Energy functions
# Abstract element energy class that implements finite differences for the gradient and hessian
# of the energy function. The energy function is defined in the derived classes, which must implement
# the energy method. The gradient and hessian methods should override the finite differences implementation.
# X is the undeformed configuration, x is the deformed configuration in a nx3 matrix, where n is the number
# of vertices in the element.
# the variable ordering for the gradient and hessian methods be in the x1, x2, x3, ..., y1, y2, y3, ... z1, z2, z3 format
class ElementEnergy(ABC):
    # constructor
    def __init__(self,X):
        self.X = X
    
    @abstractmethod
    def energy(self, x):
        return 0

    # should be overridden by the derived class, otherwise the finite difference implementation will be used
    def gradient(self, x):
        return self.gradient_fd(x)

    def hessian(self, x):
        return self.hessian_fd(x)
    
    # finite difference gradient and hessian
    def gradient_fd(self, x):
        return nd.Gradient(lambda x: self.energy(x))

    def hessian_fd(self, x):
        return nd.Hessian(lambda x: self.energy(x))
    
    # check that the gradient is correct by comparing it to the finite difference gradient
    def check_gradient(self, x):
        grad = self.gradient(x)
        h = 1e-6
        for i in range(3*x.shape[0]):
            x1 = x.copy()
            x2 = x.copy()
            x1[i] += h
            x2[i] -= h
            f1 = self.energy(x1)
            f2 = self.energy(x2)
            fd = (f1 - f2)/(2*h)
            print(grad[i], fd)

# Spring energy function for a zero-length spring, defined as E = 0.5*||x||^2, regardless of the undeformed configuration
class ZeroLengthSpringEnergy(ElementEnergy):
    def energy(self,x):
        return 0.5*np.linalg.norm(x)**2

    def gradient(self,x):
        return x.flatten()

    def hessian(self, x):  # remove x parameter since we don't need X
        n = len(x.flatten())
        return np.eye(n)
    
# Spring energy function for a spring with a rest length, defined as E = 0.5*(||x|| - l(X))^2, where l is the rest length
class SpringEnergy(ElementEnergy):
    def energy(self, x):
        return 0.5*(np.linalg.norm(x) - np.linalg.norm(self.X))**2
    
    def gradient(self, x):
        current_length = np.linalg.norm(x)
        rest_length = np.linalg.norm(self.X)
        if current_length < 1e-10:
            return np.zeros_like(x.flatten())
        return ((current_length - rest_length) * x / current_length).flatten()

    def hessian(self, x):
        n = x.size
        current_length = np.linalg.norm(x)
        rest_length = np.linalg.norm(self.X)
        if current_length < 1e-10:
            return np.eye(n)

        I = np.eye(n)
        x_normalized = x / current_length
        x_outer = np.outer(x_normalized, x_normalized)
        term1 = (current_length - rest_length) / current_length * I
        term2 = (rest_length / current_length) * (I - x_outer)

        return term1 + term2


#%% Mesh class
class FEMMesh:
    def __init__(self, mesh, energy, element_type = "edge"):
        self.mesh = mesh
        self.element_type = element_type
        self.energy = energy(mesh.vertices)  # passing mesh vertices as undeformed configuration
        self.elements = mesh.edges if element_type == "edge" else mesh.faces
        self.X = mesh.vertices
        self.nV = self.X.shape[0]
        self.nE = len(self.elements)
    def compute_energy(self,x):
        energy = 0
        for element in self.elements:
            energy += self.energy.energy(x[element])
        return energy
    
    def compute_gradient(self, x):
        grad = np.zeros_like(x)
        for element in self.elements:
            g = self.energy.gradient(x[element])
            g = g.reshape(-1, x.shape[1])   
            grad[element] += g  
        return grad.flatten()  
    
    def compute_hessian(self, x):
        I = []
        J = []
        S = []
        dim = x.shape[1]  # Dimension of the problem (2D or 3D)
        for element in self.elements:
            hess = self.energy.hessian(x[element])
            for i in range(hess.shape[0]):
                for j in range(hess.shape[1]):
                    I.append(element[i % len(element)] * dim + i // len(element))
                    J.append(element[j % len(element)] * dim + j // len(element))
                    S.append(hess[i, j])

        # Construct sparse matrix for the Hessian
        n_coords = self.nV * dim
        H = coo_matrix((S, (I, J)), shape=(n_coords, n_coords))
        H = H.toarray()
        H += np.eye(H.shape[0]) * 1e-6  # Regularization to avoid singularity
        return H


            
#%% Optimization
class MeshOptimizer:
    def __init__(self, femMesh, method='gradient'):
        self.femMesh = femMesh
        self.SearchDirection = self.Newton if method == 'newton' else self.GradientDescent
        self.LineSearch = self.BacktrackingLineSearch

    def BacktrackingLineSearch(self, x, d, alpha=1, beta=0.5):
        x0 = x.copy()
        d = d.reshape(x0.shape)
        while self.femMesh.compute_energy(x0 + alpha * d) > self.femMesh.compute_energy(x0) + 1e-4 * alpha * np.dot(self.femMesh.compute_gradient(x0).flatten(), d.flatten()):
            alpha *= beta
            if alpha < 1e-10:   
                break
        x_new = x0 + alpha * d
        return x_new, alpha
    

    def GradientDescent(self, x):
        d = self.femMesh.compute_gradient(x)
        return -d*0.1

    def Newton(self, x):
        grad = self.femMesh.compute_gradient(x)
        hess = self.femMesh.compute_hessian(x)
        
        # Adding regularization to avoid singularity
        hess += np.eye(hess.shape[0]) * 1e-6
        
        d = np.linalg.solve(hess, grad)
        return -d*0.1
        
    def step(self, x):
        d = self.SearchDirection(x)

        for pinned in pinned_vertices:
            x[pinned, :] = self.femMesh.X[pinned, :2]

        new_x, alpha = self.LineSearch(x, d)

        for pinned in pinned_vertices:
            new_x[pinned, :] = self.femMesh.X[pinned, :2] 

        return new_x, alpha



    def optimize(self, x, max_iter=100, tol=1e-6):
        for i in range(max_iter):
            # Step and get alpha
            x, alpha = self.step(x)

            # Compute energy and gradient norm
            energy = self.femMesh.compute_energy(x)
            grad_norm = np.linalg.norm(self.femMesh.compute_gradient(x))
            
            # Logging for debugging
            print(f"Iteration {i + 1}: Energy = {energy:.6f}, Alpha = {alpha:.6f}, Gradient Norm = {grad_norm:.6f}")

            # Check for convergence
            if grad_norm < tol:
                print(f"Converged after {i + 1} iterations.")
                break
        return x








def circle(N, R):
    i = np.arange(N)
    theta = i * 2 * np.pi / N
    pts = np.stack([np.cos(theta), np.sin(theta)], axis=1) * R
    seg = np.stack([i, i + 1], axis=1) % N
    return pts, seg

def CreateShape():
    pts0, seg0 = circle(30, 1.4)
    pts1, seg1 = circle(16, 0.6)
    pts = np.vstack([pts0, pts1])
    seg = np.vstack([seg0, seg1 + seg0.shape[0]])
    vertices = dict(vertices=pts, segments=seg, holes=[[0, 0]])
    tris = tr.triangulate(vertices, 'qpa0.05') 
    return tris['vertices'], tris['triangles']

def CreateShapeModifiedDonut():
    pts0, seg0 = circle(70, 1.4)  # Outer circle with ~70 points
    pts1, seg1 = circle(30, 0.6)  # Inner circle with ~30 points
    pts = np.vstack([pts0, pts1])
    seg = np.vstack([seg0, seg1 + seg0.shape[0]])
    vertices = dict(vertices=pts, segments=seg, holes=[[0, 0]])
    tris = tr.triangulate(vertices, 'qpa0.05') 
    return tris['vertices'], tris['triangles']



#%% Main program
 
V,F = CreateShapeModifiedDonut()
#%% Main program
pinned_vertices = []



def redraw():
    plt.remove("Mesh")
    mesh = vd.Mesh([V,F]).linecolor('black')
    plt.add(mesh)
    plt.remove("Points")
    plt.add(vd.Points(V[pinned_vertices,:],r=10))
    plt.render()




able_to_move = False
V_to_move = None
drag_key_pressed = False

def OnLeftButtonPress(event):
    global last_point, able_to_move
    able_to_move = False
    if event.object is None:          # mouse hits nothing, return.
        msg.text('Mouse hits nothing')
    if isinstance(event.object,vd.mesh.Mesh):          # mouse hits the mesh
        Vi = mesh.closest_point(event.picked3d, return_point_id=True)
        msg.text('Mouse hits the mesh'
                        f'\nCoordinates: {event.picked3d}'
                        f'\nPoint ID: {Vi}')
        if Vi not in pinned_vertices :
            pinned_vertices.append(Vi)
        else:
            pinned_vertices.remove(Vi)
        last_point = Vi
    redraw()

def OnRightButtonPress(event):
    global able_to_move, last_point
    able_to_move = True
    redraw()


def find_neighbors(vertex_id, radius=1):
    distances = np.linalg.norm(V[vertex_id] - V, axis=1)
    return np.where(distances < radius)[0]


def OnMouseMove(event):
    global able_to_move, mesh, V
    if able_to_move==True:
        new_pos = np.array(event.picked3d[:2])  
        displacement = new_pos - V[last_point]
        neighbors = find_neighbors(last_point, radius=0.01)
        for idx in neighbors:
            weight = max(0, 1 - np.linalg.norm(V[idx] - V[last_point]) / 0.01)
            V[idx] += displacement * weight
        mesh.points(V)   
    redraw()

femMesh = None

def OnKeyPress(event):
    global V, F, mesh, pinned_vertices, femMesh

    if event.keypress in ['A', 'a']:  # Gradient Descent with ZeroLengthSpringEnergy
        msg.text("Running Gradient Descent with ZeroLengthSpringEnergy...")
        femMesh = FEMMesh(mesh, ZeroLengthSpringEnergy)
        optimizer_gradient = MeshOptimizer(femMesh, method='gradient')
        optimize_with_visualization(optimizer_gradient, "gradient", steps=1)

    elif event.keypress in ['B', 'b']:  # Newton's Method with ZeroLengthSpringEnergy
        msg.text("Running Newton's Method with ZeroLengthSpringEnergy...")
        femMesh = FEMMesh(mesh, ZeroLengthSpringEnergy)
        optimizer_newton = MeshOptimizer(femMesh, method='newton')
        optimize_with_visualization(optimizer_newton, "newton", steps=1)


    elif event.keypress in ['C', 'c']:  # Gradient Descent with SpringEnergy
        msg.text('Running Gradient Descent with SpringEnergy...')
        femMesh = FEMMesh(mesh, SpringEnergy)
        optimizer_gradient = MeshOptimizer(femMesh, method='gradient')
        optimize_with_visualization(optimizer_gradient, "gradient", steps=1)

    elif event.keypress in ['D', 'd']:  # Newton's Method with SpringEnergy
        msg.text('Running Newton\'s Method with SpringEnergy...')
        femMesh = FEMMesh(mesh, SpringEnergy)
        optimizer_newton = MeshOptimizer(femMesh, method='newton')
        optimize_with_visualization(optimizer_newton, "newton", steps=1)
    
    elif event.keypress in ['R', 'r']:  # Reset mesh
        msg.text('Resetting mesh...')
        V, F = CreateShapeModifiedDonut()  # Generate a new shape
        pinned_vertices.clear()  # Clear pinned vertices
        redraw()  # Redraw the reset mesh

    else:
        print(f"Key {event.keypress} is not assigned to any function.")




def optimize_with_visualization(optimizer, method_name, steps):
    print(f"Starting {method_name} optimization...")
    global V
    print(f"Before optimization: {V[:5]}")  # Print first 5 vertices

    # Use optimize to perform multiple steps
    V = optimizer.optimize(V, max_iter=steps)  # Perform optimization

    print(f"After optimization: {V[:5]}")  # Print first 5 vertices
    redraw()  # Update mesh visualization
    return V
  
plt = vd.Plotter()
plt.add(msg)
plt.add(usage)
plt.add_callback('LeftButtonPress', OnLeftButtonPress) # add Keyboard callback
plt.add_callback('MouseMove', OnMouseMove) # add Keyboard callback
plt.add_callback('RightButtonPress', OnRightButtonPress) # add Keyboard callback
plt.add_callback('KeyPress', OnKeyPress) # add Keyboard callback

mesh = vd.Mesh([V, F]).linecolor('black')  # Set the mesh color to red



 

plt.add(mesh)
plt.add(vd.Points(V[pinned_vertices,:]))

plt.user_mode('2d')
plt.show(interactive=True)
plt.close()


# %%
