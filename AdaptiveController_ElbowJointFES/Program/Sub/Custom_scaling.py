class CustomScaler:
    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val
        
    def fit(self, X):
        return self
    
    def transform(self, X):
        scaled = (X - self.min_val) / (self.max_val - self.min_val)
        return scaled
    
    def fit_transform(self, X):
        self.fit(X)
        return self.transform(X)
    
class CustomInverseScaler:
    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val
        self.data_range = max_val - min_val

    def fit(self, X_scaled):
        return self
    
    def transform(self, X_scaled):
        X_original = X_scaled * self.data_range + self.min_val
        return X_original
        
    def inverse_transform(self, X_scaled):
        self.fit(X_scaled)
        return self.transform(X_scaled)