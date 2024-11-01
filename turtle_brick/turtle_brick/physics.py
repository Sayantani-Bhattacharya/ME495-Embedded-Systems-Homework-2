class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick_pose, gravity=9.81, radius=2.0, dt=1/250.0):
        """
        Initialize the world.

        Args:
        brick - The (x,y,z) location of the brick
        gravity - the acceleration due to gravity in m/s^2
        radius - the radius of the platform
        dt - timestep in seconds of the physics simulation
        """
        self.x = brick_pose[0]
        self.y = brick_pose[1]
        self.z = brick_pose[2]
        self.gravity = gravity
        self.platform_radius = radius
        self.dt = dt
        self.initial_vel = 0.0 

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
            (x,y,z) location of the brick
        """      
        return (self.x,self.y,self.z)

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
           location - the (x,y,z) location of the brick
        """
        self.x = location[0]
        self.y = location[1]
        self.z = location[2]

    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        updated_brick_pose = [0.0, 0.0, 0.0]
        # From equations of motion
        self.z = self.z - ( ( self.dt**2 ) * self.gravity/2.0 + self.initial_vel * self.dt)
        updated_brick_pose[0] = self.x
        updated_brick_pose[1] = self.y
        updated_brick_pose[2] = self.z
        self.initial_vel =  self.initial_vel + self.gravity * self.dt
        return updated_brick_pose
        