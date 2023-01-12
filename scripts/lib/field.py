import matplotlib.image as mpimg

def drawField(ax):
  ax.imshow(mpimg.imread("lib/field.png"), origin="upper", extent=(0, 16.46, 0, 8.23))