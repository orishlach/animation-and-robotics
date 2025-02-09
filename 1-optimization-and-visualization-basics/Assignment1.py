#%% imports
import vedo as vd
import numpy as np
from vedo.pyplot import plot
from vedo import Latex

""" """

vd.settings.default_backend= 'vtk'

#%% Callbacks
msg = vd.Text2D(pos='bottom-left', font="VictorMono") # an empty text
fig = None
fig_grad = None
fig_newton = None

def objective(X):
    x, y = X[0], X[1]
    return np.sin(2*x*y) * np.cos(3*y)/2+1/2

################# plot
def UpdatePlot(x_val, y_val): 
    global fig

    if fig is not None:
        plt.at(0).remove(fig)

    fig = plot(
        x_val, y_val,
        title= "Function Values Along Path",
        xtitle='Point Index',
        ytitle='Function Value',
        ).clone2d(pos='top-left', size=0.7)

    plt.at(0).add(fig)

def UpdateGrad(x_val, y_val): 
    global fig_grad
    if fig_grad is not None:
        plt.at(1).remove(fig_grad)

    fig_grad = plot(
        x_val, y_val,
        title= "Grad",
        xtitle='x',
        ytitle='y',
        ).clone2d('top-right', size=0.6)

    plt.at(1).add(fig_grad)


def UpdateNewton(x_val, y_val): 
    global fig_newton
    if fig_newton is not None:
        plt.at(1).remove(fig_newton)

    fig_newton = plot(
        x_val, y_val,
        title= "Function Values Along Path",
        xtitle='x',
        ytitle='y',
        ).clone2d('bottom-right', size=0.6)

    plt.at(1).add(fig_newton)



################# 

def OnMouseMove(pt):                
    global Xi, X
    if pt is None: return              
    X   = np.array([pt[0],pt[1],objective([pt[0],pt[1]])])  # X = (x,y,e(x,y))
    Xi = np.append(Xi,[X],axis=0)             # append to the list of points
    if len(Xi) > 1:               # need at least two points to compute a distance
        txt =(
            f"X:  {vd.precision(X,2)}\n"
            f"dX: {vd.precision(Xi[-1,0:2] - Xi[-2,0:2],2)}\n"
            f"dE: {vd.precision(Xi[-1,2] - Xi[-2,2],2)}\n"
        )

        UpdatePlot(range(len(Xi)), Xi[:, 2])
    else:
        txt = f"X: {vd.precision(X,2)}"
    msg.text(txt)                    # update text message
 
    fp = fplt3d[0].flagpole(txt, point=X,s=0.08, c='k', font="Quikhand")
    fp.follow_camera()                 # make it always face the camera
    plt.at(0).remove("FlagPole") # remove the old flagpole

    plt.at(0).add(fp) # add the new flagpole 
    plt.at(0).render()   # re-render the scene

def OnKeyPress(evt):               ### called every time a key is pressed
    if evt.keypress in ['c', 'C']: # reset Xi and the arrows
        Xi = np.empty((0, 3))
        plt.at(0).remove("Arrow").render()

def OnSliderAlpha(widget, event): ### called every time the slider is moved
    val = widget.value         # get the slider value
    fplt3d[0].alpha(val)       # set the alpha (transparency) value of the surface
    fplt3d[1].alpha(val)       # set the alpha (transparency) value of the isolines

def OnMouseRightClick(evt):
    if evt.object is None:
        return
        
    pt = evt.picked3d
    
    t = np.linspace(0, 4*np.pi, 100)
    radius = np.linspace(0, 0.2, 100)
    x = pt[0] + radius * np.cos(t)
    y = pt[1] + radius * np.sin(t)
    z = [objective([xi, yi]) for xi, yi in zip(x, y)]
    
    point = vd.Point(pt, c='red', r=15)
    plt.at(0).add(point)
    
    for i in range(len(t)):
        point.pos([x[i], y[i], z[i]])
        plt.at(0).render()
    
    plt.at(0).remove(point).render()

def OnLeftMouseClick(evt):
    global X
    if plt.renderers.index(plt.renderer) != 0:
        return 
    OnMouseMove(evt.picked3d)


X_grad = None
X_newton = None
X_grad_i = np.empty((0, 3))
X_newton_i = np.empty((0, 3))

def gradient(obj, ename):
    global X, X_grad_i 
    if plt.renderers.index(plt.renderer) != 1:
        return 

    new_point = step(objective, X[:2], gradient_direction)  
    X = new_point
    if new_point is None:
        return 

    new_point_3d = np.array([new_point[0], new_point[1], objective(new_point[:2])])  
    X_grad_i = np.append(X_grad_i, [new_point_3d], axis=0)  
    UpdateGrad(range(len(X_grad_i)), X_grad_i[:, 2])
    OnMouseMove(new_point_3d)



def newton(obj, ename):
    global X, X_newton_i
    if plt.renderers.index(plt.renderer) != 1:
        return  
    new_point = step(objective, X, Newton_direction)  
    X = new_point
    if new_point is None:
        return 
    new_point_2d = np.array([new_point[0],new_point[1],objective([new_point[0],new_point[1]])])  # X = (x,y,e(x,y))
    X_newton_i = np.append(X_newton_i,[new_point_2d],axis=0)             
    UpdateNewton(range(len(X_newton_i)), X_newton_i[:, 2])
    OnMouseMove(new_point) 



#%% Optimization functions
def gradient_fd(func, X, h=0.001): # finite difference gradient
    x, y = X[0], X[1]
    gx = (func([x+h, y]) - func([x-h, y])) / (2*h)
    gy = (func([x, y+h]) - func([x, y-h])) / (2*h)
    return gx, gy

def Hessian_fd(func, X, h=0.001): # finite difference Hessian
    x, y = X[0], X[1]
    gxx = (func([x+h, y]) - 2*func([x, y]) + func([x-h, y])) / h**2
    gyy = (func([x, y+h]) - 2*func([x, y]) + func([x, y-h])) / h**2
    gxy = (func([x+h, y+h]) - func([x+h, y-h]) - func([x-h, y+h]) + func([x-h, y-h])) / (4*h**2)
    H = np.array([[gxx, gxy], [gxy, gyy]])
    return H

def gradient_direction(func, X): # compute gradient step direction
    g = gradient_fd(func, X)
    return np.array([-g[0], -g[1]])

def Newton_direction(func, X):   # compute Newton step direction
    g = gradient_fd(func, X)
    H = Hessian_fd(func, X)
    d = -np.linalg.solve(H, np.array(g))
    return np.array(d[0],d[1])

def line_search(func, X, d): 
    alpha = 1.0
    while func(X + d*alpha) < func(X):  # If the function value increases, reduce alpha
        alpha *= 0.5                                # by half and try again
    return alpha

def step(func, X, search_direction_function):
    d = search_direction_function(func, X)
    alpha = line_search(func, X, d)
    return X + d*alpha

def optimize(func, X, search_direction_function, tol=1e-6, iter_max=10):
    for i in range(iter_max):
        X = step(func, X, search_direction_function)
        if np.linalg.norm(gradient_fd(func, X)) < tol:
            break
    return X

Xi = np.empty((0, 3))
# test the optimization functions
X = optimize(objective, [0.6, 0.6], Newton_direction, tol=1e-6, iter_max=100)


#%% Plotting
plt = vd.Plotter(bg2='lightblue', shape=(1,2))  # Create the plotter
fplt3d = plot(lambda x,y: objective([x,y]), c='terrain')      # create a plot from the function e. fplt3d is a list containing surface mesh, isolines, and axis
fplt2d = fplt3d.clone()            # clone the plot to create a 2D plot

# Add a button to the plot:
plt.at(1).add_button(
    gradient,
    pos=(0.2, 0.8),
    states=["click to Gradient"], 
    c=["w"],     
    bc=["dg"],  
    font="courier",   
    size=20,          
    bold=True,       
    italic=False,    
)
plt.at(1).add_button(
    newton,
    pos=(0.2, 0.3),
    states=["click to Newton"], 
    c=["w"],    
    bc=["dv"], 
    font="courier",  
    size=20,         
    bold=True,        
    italic=False,     
)


fplt2d[0].lighting('off')          # turn off lighting for the 2D plot
fplt2d[0].vertices[:,2] = 0        # set the z-coordinate of the mesh to 0
fplt2d[1].vertices[:,2] = 0        # set the z-coordinate of the isolines to 0

plt.at(0).add(fplt3d)              # add the 3D plot to the plotter
plt.at(0).add(fplt2d)              # add the 2D plot to the plotter
plt.at(0).add(msg)


#plt.add_callback('mouse move', OnMouseMove) # add Mouse move callback
plt.add_callback('key press', OnKeyPress) # add Keyboard callback
plt.add_callback('mouse right click', OnMouseRightClick) # add right click callback
plt.add_callback('mouse left click', OnLeftMouseClick) # add left click callback
plt.add_slider(OnSliderAlpha,0.,1.,1., title="Alpha",pos=([0.2,0.1], [0.4,0.1])) # add a slider for the alpha value of the surface

plt.show(__doc__ ,interactive=True, viewup='z')
plt.close()
# %%
