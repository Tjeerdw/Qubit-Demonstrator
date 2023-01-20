import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Ellipse,Path,PathPatch,Arrow
from matplotlib.text import Text
from matplotlib import transforms

orb_radius = 1.
wheel_radius = .2
wheel_angle = .19445 * np.pi

class Quaternion:
    @classmethod
    def from_vector(cls, u):
        return cls([0, *u])

    @classmethod
    def from_u_theta(cls, u, theta):
        return cls([np.cos(theta/2), *(np.sin(theta/2)*np.array(u))])

    def __init__(self, q):
        self.q = np.array(q)
        assert self.q.size == 4

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(
                [
                    self.q[0] * other.q[0] - np.dot(self.q[1:], other.q[1:]),
                    *(self.q[0] * other.q[1:] + other.q[0] * self.q[1:] + np.cross(self.q[1:], other.q[1:]))
                ]
            )
        else:
            return Quaternion(self.q * other)

    def __truediv__(self, other):
        if isinstance(other, Quaternion):
            return self * other.inverse()
        else:
            return Quaternion(self.q / other)

    def __str__(self):
        return "{} + {}i + {}j + {}k".format(*self.q)

    def to_vector(self):
        assert abs(self.q[0]) < 1e-8
        return self.q[1:]

    def to_real(self):
        assert np.linalg.norm(self.q[1:]) < 1e-8
        return self.q[0]

    def conjugate(self):
        return Quaternion([self.q[0], *(-self.q[1:])])

    def norm(self):
        return np.sqrt((self * self.conjugate()).to_real())

    def inverse(self):
        return self.conjugate() / np.square(self.norm())

    def to_rotation_matrix(self):
        return np.column_stack([
            (self * Quaternion.from_vector([0]*i + [1] + [0]*(2-i)) * self.inverse()).to_vector() for i in range(3)
        ])

class Point3D:
    def __init__(self, point):
        if isinstance(point, Point3D): self.point = point.point.copy()
        else: self.point = np.array(point).flatten()

    def project_view(self, view):
        assert isinstance(view, Quaternion)
        return Point3D(view.to_rotation_matrix() @ self.point)

    def to_quaternion(self):
        return Quaternion([0, *self.point])

class Arrow3D:
    def __init__(self, start, end):
        self.start = Point3D(start)
        self.end = Point3D(end)

    def project_view(self, view):
        return Line3D(self.start.project_view(view), self.end.project_view(view))

    def to_patch(self, view, **kwargs):
        n = self.project_view(view)
        return Arrow(n.start.point[1], n.start.point[2], n.end.point[1] - n.start.point[1], n.end.point[2] - n.start.point[2], zorder = .5*(n.start.point[0] + n.end.point[0]), **kwargs)

class Line3D:
    def __init__(self, start, end):
        self.start = Point3D(start)
        self.end = Point3D(end)

    def project_view(self, view):
        return Line3D(self.start.project_view(view), self.end.project_view(view))

    def calc_args(self, view):
        n = self.project_view(view)
        return (np.row_stack([n.start.point[1:], n.end.point[1:]]), .5*(n.start.point[0] + n.end.point[0]))

    def to_patch(self, view, **kwargs):
        points, zorder = self.calc_args(view)
        return PathPatch(Path(points), zorder = zorder, **kwargs)

class Text3D:
    def __init__(self, point, s):
        self.point = Point3D(point)
        self.s = s

    def project_view(self, view):
        return Text3D(self.point.project_view(view), self.s)

    def calc_args(self, view):
        n = self.project_view(view)
        return n.point.point[1], n.point.point[2], n.s, n.point.point[0]

    def to_text(self, view, **kwargs):
        x, y, s, zorder = self.calc_args(view)
        return Text(x = x, y = y, text = s, zorder = zorder, **kwargs)

class Sphere3D:
    def __init__(self, center, radius):
        assert radius > 0
        self.center = Point3D(center)
        self.radius = radius

    def project_view(self, view):
        return Sphere3D(self.center.project_view(view), self.radius)

    def to_patch(self, view, **kwargs):
        n = self.project_view(view)
        return Circle(n.center.point[1:], self.radius, zorder = n.center.point[0], **kwargs)

class Circle3D:
    def __init__(self, center, normal, radius):
        assert radius > 0
        self.center = Point3D(center)
        self.normal = np.array(normal).flatten()
        self.normal /= np.linalg.norm(self.normal)
        self.radius = radius

    def project_view(self, view):
        return Circle3D(self.center.project_view(view), view.to_rotation_matrix() @ self.normal, self.radius)

    def calc_args(self, view, **kwargs):
        n = self.project_view(view)
        long_axis_3d = np.cross(np.array([1,0,0]), n.normal)
        long_axis_3d = long_axis_3d / np.linalg.norm(long_axis_3d) * n.radius
        short_axis_3d = np.cross(long_axis_3d, n.normal)
        long_axis_2d = long_axis_3d[1:]
        short_axis_2d = short_axis_3d[1:]
        angle = np.arctan2(*reversed(long_axis_2d)) - np.pi/2
        return (n.center.point[1:], 2*np.linalg.norm(short_axis_2d), 2*np.linalg.norm(long_axis_2d), angle/np.pi*180, n.center.point[0])
        # return Ellipse(n.center.point[1:], 2*np.linalg.norm(short_axis_2d), 2*np.linalg.norm(long_axis_2d), angle=angle/np.pi*180, zorder = n.center.point[0], **kwargs)

    def to_patch(self, view, **kwargs):
        center, hor_diam, ver_diam, angle, zorder = self.calc_args(view)
        return Ellipse(center, hor_diam, ver_diam, angle = angle, zorder = zorder, **kwargs)

class Orb:
    def __init__(self):
        self.rot = Quaternion((1,0,0,0))
        self.wheel_angles = np.zeros(3)
        wheel_vectors = [orb_radius * np.array([np.sin(wheel_angle) * np.cos(phi), np.sin(wheel_angle) * np.sin(phi), -np.cos(wheel_angle)]) for phi in [0., 2.*np.pi/3, 4.*np.pi/3]]
        wheel_normal_vectors = list(map(lambda x : x / np.linalg.norm(x), [orb_radius * np.array([np.cos(wheel_angle) * np.cos(phi), np.cos(wheel_angle) * np.sin(phi), np.sin(wheel_angle)]) for phi in [0., 2.*np.pi/3, 4.*np.pi/3]]))
        wheel_direction_vectors = list(map(lambda x : x / np.linalg.norm(x), [np.cross(v,n) for v,n in zip(wheel_vectors, wheel_normal_vectors)]))
        self.wheel_vectors = wheel_vectors
        self.coeff_matrix = np.row_stack([np.cross(v,d).flatten() for v,d in zip(wheel_vectors, wheel_direction_vectors)])
        self.inv_coeff_matrix = np.linalg.inv(self.coeff_matrix)
        self.stop_rotation()

    def calc_u_theta(self, fact = 1.):
        u = (self.inv_coeff_matrix @ self.wheel_speeds).flatten()
        theta = np.linalg.norm(u)
        if abs(theta) < 1e-8:
            u = [0,0,1]
            theta = 0
        else:
            u /= theta
        theta *= (wheel_radius / orb_radius) * fact
        return u,theta

    def update_rotation(self, fact = 1.):
        u,theta = self.calc_u_theta(fact = fact)
        drot = Quaternion.from_u_theta(u, theta)
        self.rot = drot * self.rot
        self.wheel_angles += self.wheel_speeds * fact

    def stop_rotation(self):
        self.wheel_speeds = np.zeros(3)

    def set_rotation(self, u, theta):
        self.wheel_speeds = (self.coeff_matrix @ u) * theta

    def adjust_wheel_speed(self, i, dv):
        self.wheel_speeds[i] += dv

    def draw_interactive(self):
        fig = plt.figure(figsize = (5,5))
        fig.add_axes(InteractiveOrb(self))
        return fig

class InteractiveOrb(plt.Axes):
    def __init__(self, orb = None, view = Quaternion([1,0,0,0]), fig = None, rect = [0, 0.16, 1, 0.84], **kwargs):
        # Determine what orb should be drawn
        if orb is None:
            self.orb = Orb()
        elif isinstance(orb, Orb):
            self.orb = orb
        else:
            self.orb = Orb(orb)

        # Get the correct figure handle, if not supplied
        if fig is None:
            fig = plt.gcf()

        # Disable the default keypress callback
        callbacks = fig.canvas.callbacks.callbacks
        del callbacks['key_press_event']

        # Call the normal axis initialization function with suitable parameters
        kwargs.update(
            dict(
                aspect = kwargs.get('aspect', 'equal'),
                xlim = kwargs.get('xlim', (-2.,2.)),
                ylim = kwargs.get('ylim', (-2.,2.)),
                frameon = kwargs.get('frameon', False),
                xticks = kwargs.get('xticks', []),
                yticks = kwargs.get('yticks', [])
            )
        )
        super(InteractiveOrb, self).__init__(fig, rect, **kwargs)
        self.xaxis.set_major_formatter(plt.NullFormatter())
        self.yaxis.set_major_formatter(plt.NullFormatter())
        self._start_xlim = kwargs['xlim']
        self._start_ylim = kwargs['ylim']

        # Define movement for up/down arrows or up/down mouse movement
        self._ax_UD = (0, 1, 0)
        self._step_UD = 0.01

        # Define movement for left/right arrows or left/right mouse movement
        self._ax_LR = (0, 0, -1)
        self._step_LR = 0.01

        self._ax_LR_alt = (1, 0, 0)

        # Internal state variable
        self._shift = False  # shift key pressed
        self._objects = None

        # Set up the viewing angle
        self._view = view

        # Draw the orb
        self._draw_orb()

        # Connect the GUI events
        self.figure.canvas.mpl_connect('key_press_event', self._key_press)
        self.figure.canvas.mpl_connect('key_release_event', self._key_release)

        # Set the timer for redrawal of the orb
        # Moved to the end of the source, bug?

    def _update_timestep(self):
        self.orb.update_rotation()
        self._draw_orb()

    def _draw_orb(self):
        if self._objects is None:
            # Allocate the list of objects to be drawn
            self._objects = []

            # Draw the arrow inside the orb
            arrow = Arrow3D([0,0,-1], [0,0,1])
            self._objects.append((arrow, arrow.to_patch(self._view, color = 'orange'), self._update_middle_arrow))
            indicator_circle = Circle3D([0,0,orb_radius], [0,0,1.], .02*orb_radius)
            self._objects.append((indicator_circle, indicator_circle.to_patch(self._view, color = 'orange'), self._update_indicator_circle))

            # Draw the orb
            ball = Sphere3D([0,0,0], 1)
            self._objects.append((ball, ball.to_patch(self._view, color = 'gray', alpha = .8), self._update_dummy))

            # Draw the wheels
            front_wheel = Circle3D((orb_radius + wheel_radius)*np.array([np.sin(wheel_angle),0,-np.cos(wheel_angle)]), np.array([np.cos(wheel_angle),0,np.sin(wheel_angle)]), wheel_radius)
            back_left_wheel = Circle3D((orb_radius + wheel_radius)*np.array([np.sin(wheel_angle)*np.cos(2*np.pi/3),-np.sin(wheel_angle)*np.sin(2*np.pi/3),-np.cos(wheel_angle)]), np.array([np.cos(wheel_angle)*np.cos(2*np.pi/3),-np.cos(wheel_angle)*np.sin(2*np.pi/3),np.sin(wheel_angle)]), wheel_radius)
            back_right_wheel = Circle3D((orb_radius + wheel_radius)*np.array([np.sin(wheel_angle)*np.cos(2*np.pi/3),np.sin(wheel_angle)*np.sin(2*np.pi/3),-np.cos(wheel_angle)]), np.array([np.cos(wheel_angle)*np.cos(2*np.pi/3),np.cos(wheel_angle)*np.sin(2*np.pi/3),np.sin(wheel_angle)]), wheel_radius)
            self._objects.append((front_wheel, front_wheel.to_patch(self._view, color = 'blue'), self._update_ellipse))
            self._objects.append((back_left_wheel, back_left_wheel.to_patch(self._view, color = 'blue'), self._update_ellipse))
            self._objects.append((back_right_wheel, back_right_wheel.to_patch(self._view, color = 'blue'), self._update_ellipse))
            front_wheel_arrow = Arrow3D((orb_radius + 1.2*wheel_radius)*np.array([np.sin(wheel_angle),0,-np.cos(wheel_angle)]), (orb_radius + .8*wheel_radius)*np.array([np.sin(wheel_angle),0,-np.cos(wheel_angle)]))
            back_left_wheel_arrow = Arrow3D((orb_radius + 1.2*wheel_radius)*np.array([np.sin(wheel_angle)*np.cos(2*np.pi/3),-np.sin(wheel_angle)*np.sin(2*np.pi/3),-np.cos(wheel_angle)]), (orb_radius + .8*wheel_radius)*np.array([np.sin(wheel_angle)*np.cos(2*np.pi/3),-np.sin(wheel_angle)*np.sin(2*np.pi/3),-np.cos(wheel_angle)]))
            back_right_wheel_arrow = Arrow3D((orb_radius + 1.2*wheel_radius)*np.array([np.sin(wheel_angle)*np.cos(2*np.pi/3),np.sin(wheel_angle)*np.sin(2*np.pi/3),-np.cos(wheel_angle)]), (orb_radius + .8*wheel_radius)*np.array([np.sin(wheel_angle)*np.cos(2*np.pi/3),np.sin(wheel_angle)*np.sin(2*np.pi/3),-np.cos(wheel_angle)]))
            front_wheel_arrow.id = 0
            back_left_wheel_arrow.id = 1
            back_right_wheel_arrow.id = 2
            self._objects.append((front_wheel_arrow, front_wheel_arrow.to_patch(self._view, color = 'red', width = .1), self._update_indicator_arrow))
            self._objects.append((back_left_wheel_arrow, back_left_wheel_arrow.to_patch(self._view, color = 'red', width = .1), self._update_indicator_arrow))
            self._objects.append((back_right_wheel_arrow, back_right_wheel_arrow.to_patch(self._view, color = 'red', width = .1), self._update_indicator_arrow))

            # Draw the ground
            ground = Circle3D(np.array([0.,0.,-1.1]), np.array([0.,0.,1.]), .5)
            self._objects.append((ground, ground.to_patch(self._view, color = 'black'), self._update_ellipse))

            # Draw the axes
            x_axis_tail = Line3D([-1,0,0],[-1.2,0,0])
            y_axis_tail = Line3D([0,-1,0],[0,-1.2,0])
            self._objects.append((x_axis_tail, x_axis_tail.to_patch(self._view, color = 'red'), self._update_line))
            self._objects.append((y_axis_tail, y_axis_tail.to_patch(self._view, color = 'red'), self._update_line))
            x_axis_head = Line3D([1,0,0],[1.2,0,0])
            y_axis_head = Line3D([0,1,0],[0,1.2,0])
            z_axis_head = Line3D([0,0,1],[0,0,1.2])
            self._objects.append((x_axis_head, x_axis_head.to_patch(self._view, color = 'red'), self._update_line))
            self._objects.append((y_axis_head, y_axis_head.to_patch(self._view, color = 'red'), self._update_line))
            self._objects.append((z_axis_head, z_axis_head.to_patch(self._view, color = 'red'), self._update_line))
            x_axis_label = Text3D([1.3,0,0], 'x')
            y_axis_label = Text3D([0,1.3,0], 'y')
            z_axis_label = Text3D([0,0,1.3], 'z')
            self._objects.append((x_axis_label, x_axis_label.to_text(self._view, color = 'red'), self._update_text))
            self._objects.append((y_axis_label, y_axis_label.to_text(self._view, color = 'red'), self._update_text))
            self._objects.append((z_axis_label, z_axis_label.to_text(self._view, color = 'red'), self._update_text))

            # Add the labels in the corner
            self.label_text = Text(x = -2.2, y = -2.5, text = "w1 = {}\nw2 = {}\nw3 = {}\naxis = {}".format(0,0,0,np.array([0,0,1])))
            self._add_text(self.label_text)

            # Add the objects to the internal variable
            for obj,patch,update in self._objects:
                if patch is None: continue
                if isinstance(patch, Text):
                    self._add_text(patch)
                else:
                    self.add_patch(patch)
        else:
            for obj,patch,update in self._objects:
                update(obj,patch)
            self.label_text.set_text("w1 = {}\nw2 = {}\nw3 = {}\naxis = {}".format(*np.round(self.orb.wheel_speeds,3), '[' + ','.join(map(str,np.round(self.orb.calc_u_theta()[0],3))) + ']'))
        self.figure.canvas.draw()

    def _update_middle_arrow(self, obj, patch):
        end = Point3D(self.orb.rot.to_rotation_matrix() @ obj.end.point).point
        start = -end
        self._update_arrow(patch, start, end)

    def _update_arrow(self, patch, start, end, width = 1.0):
        start = Point3D(start).project_view(self._view).point
        end = Point3D(end).project_view(self._view).point
        x,y = start[1:]
        dx,dy = (end-start)[1:]

        # Ripped from the source code of Arrow
        L = np.hypot(dx, dy)
        if L != 0:
            cx = dx / L
            sx = dy / L
        else:
            # Account for division by zero
            cx, sx = 0, 1
        trans1 = transforms.Affine2D().scale(L, width)
        trans2 = transforms.Affine2D.from_values(cx, sx, -sx, cx, 0.0, 0.0)
        trans3 = transforms.Affine2D().translate(x, y)
        trans = trans1 + trans2 + trans3
        patch._patch_transform = trans.frozen()
        # End rip

        # Update the zorder
        patch.set_zorder(.5 * (start[0] + end[0]))

    def _update_ellipse(self, obj, patch):
        center, hor_diam, ver_diam, angle, zorder = obj.calc_args(self._view)
        patch.center = center
        patch.width, patch.height = hor_diam, ver_diam
        patch.angle = angle
        patch._recompute_transform()
        patch.set_zorder(zorder)

    def _update_line(self, obj, patch):
        points, zorder = obj.calc_args(self._view)
        patch._path.vertices = points
        patch.set_zorder(zorder)

    def _update_text(self, obj, patch):
        x, y, s, zorder = obj.calc_args(self._view)
        patch.set_x(x)
        patch.set_y(y)
        patch.set_zorder(zorder)

    def _update_dummy(self, obj, patch):
        pass

    def _update_indicator_circle(self, obj, patch):
        center = Point3D(self.orb.rot.to_rotation_matrix() @ np.array([0,0,orb_radius])).point
        obj.center = Point3D(center)
        obj.normal = center / np.linalg.norm(center)
        self._update_ellipse(obj, patch)

    def _update_indicator_arrow(self, obj, patch):
        i = obj.id
        v = self.orb.wheel_vectors[i] * (1. + wheel_radius / orb_radius)
        angle = self.orb.wheel_angles[i]
        pos = -v / np.linalg.norm(v) * .2 * wheel_radius
        side = np.cross(pos, np.array([0,0,1]))
        side = side / np.linalg.norm(side) * .2 * wheel_radius
        end = v + np.cos(angle) * pos + np.sin(angle) * side
        start = v - np.cos(angle) * pos - np.sin(angle) * side
        self._update_arrow(patch, start, end, width = .1)

    def rotate(self, rot):
        self._view = rot.inverse() * self._view

    def _key_press(self, event):
        if event.key == 'shift':
            self._shift = True
        elif event.key == 'right':
            if self._shift:
                ax_LR = self._ax_LR_alt
            else:
                ax_LR = self._ax_LR
            self.rotate(Quaternion.from_u_theta(ax_LR, 5 * self._step_LR))
        elif event.key == 'left':
            if self._shift:
                ax_LR = self._ax_LR_alt
            else:
                ax_LR = self._ax_LR
            self.rotate(Quaternion.from_u_theta(ax_LR, -5 * self._step_LR))
        elif event.key == 'up': self.rotate(Quaternion.from_u_theta(self._ax_UD, 5 * self._step_UD))
        elif event.key == 'down': self.rotate(Quaternion.from_u_theta(self._ax_UD, -5 * self._step_UD))
        elif event.key == 'x': self.orb.set_rotation([1,0,0],1)
        elif event.key == 'y': self.orb.set_rotation([0,1,0],1)
        elif event.key == 'z': self.orb.set_rotation([0,0,1],1)
        elif event.key == 'X': self.orb.set_rotation([1,0,0],-1)
        elif event.key == 'Y': self.orb.set_rotation([0,1,0],-1)
        elif event.key == 'Z': self.orb.set_rotation([0,0,1],-1)
        elif event.key == 's': self.orb.stop_rotation()
        elif event.key == '1': self.orb.adjust_wheel_speed(0, .05)
        elif event.key == '2': self.orb.adjust_wheel_speed(1, .05)
        elif event.key == '3': self.orb.adjust_wheel_speed(2, .05)
        elif event.key == '!': self.orb.adjust_wheel_speed(0, -.05)
        elif event.key == '@': self.orb.adjust_wheel_speed(1, -.05)
        elif event.key == '#': self.orb.adjust_wheel_speed(2, -.05)

    def _key_release(self, event):
        if event.key == 'shift':
            self._shift = False

if __name__ == "__main__":
    print("Welcome to the orb simulator!")
    print("In order to adjust the viewing angle, use the arrow keys and try combining with shift.")
    print("In order to adjust the speed of the wheels, use the keys 1, 2 and 3 on the keypad, in combination with shift.")
    print("In order to set the rotation around one of the cartestian axes, hit x, y or z, and try combining with shit.")
    print("To stop all rotation, hit s.")

    orb = Orb()
    print("Coefficient matrix to be used in the orb's code:")
    print(orb.coeff_matrix)
    
    fig = orb.draw_interactive()
    timer = fig.canvas.new_timer(interval = 50)
    timer.add_callback(fig.gca()._update_timestep)
    timer.start()
    plt.show()
