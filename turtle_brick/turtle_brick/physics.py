class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        brick - The (x,y,z) location of the brick
        gravity - the acceleration due to gravity in m/s^2
        radius - the radius of the platform
        dt - timestep in seconds of the physics simulation
        """
        self.x = 0
        self.y = 0 
        self.z = 0
        self.gravity = 9.81
        self.platform_radius = 2.0
        self.dt = 0.1

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
            (x,y,z) location of the brick
        """      
        return {self.x,self.y,self.z}

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
           location - the (x,y,z) location of the brick
        """
        self.x = location.x
        self.y = location.y
        self.z = location.z

    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        # From equations of motion
        self.z += ( self.dt**2 ) * self.gravity/2.0
        