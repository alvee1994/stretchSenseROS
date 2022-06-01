"""Classes encapsulating a particular user."""

class User:
    """This class encapsulates a particular user.
    
    Attributes:
        name:
            A string containing the user's username, used for finding their
            default theta file.
    """
    
    def __init__(self, name: str):
        self.name: str = name