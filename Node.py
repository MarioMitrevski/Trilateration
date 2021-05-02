class Node:
    x = 0
    y = 0
    is_anchor = False
    is_found = False
    x_prim = None
    y_prim = None
    relevancy = 0

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "X: " + str(self.x) + " Y: " + str(self.y) + " X predicted: " + str(self.x_prim) + " Y predicted: " + str(self.y_prim) + " Anchor: " + str(self.is_anchor) + " Is found: " + str(self.is_found) + " Relevancy: " + str(self.relevancy)
