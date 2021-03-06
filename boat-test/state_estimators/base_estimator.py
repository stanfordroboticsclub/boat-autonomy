class BaseEstimator(object):
    """all estimators will be a subclass of this class"""

    def __init__(self, name):
        super(BaseEstimator, self).__init__()
        self.name = name

    def estimate(self, raw_state):
        pass


class IdentityEstimator(BaseEstimator):
    """simply returns the raw state as is"""

    def __init__(self):
        super(IdentityEstimator, self).__init__("Identity Estimator")

    def estimate(self, raw_state):
        return raw_state
