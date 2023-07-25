
#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from sklearn.decomposition import PCA
import threading, warnings, matplotlib
warnings.simplefilter("ignore", UserWarning)


class Plot():

    def __move_figure(self, f, x, y):
        """
        Move figure's upper left corner to pixel [x, y].
        """
        backend = matplotlib.get_backend()
        if   (backend == 'TkAgg'): f.canvas.manager.window.wm_geometry(f'+{x}+{y}')
        elif (backend == 'WXAgg'): f.canvas.manager.window.SetPosition((x, y))
        else: f.canvas.manager.window.move(x, y) # works for QT and GTK


    def __initialize_figure(self, title, x, y):
        """
        Initialize the figure frame, setting title and location.
        Parameters:
            - title: title of the plot
            - x: X-coordinate of the screen
            - y: Y-coordinate of the screen
            - disable_toolbar: Disable navigation toolbar
        Return:
            figure and axis object
        """

        # Disable toolbar
        if (self.__disable_toolbar): matplotlib.rcParams['toolbar'] = 'None'
        # Initialize the figure
        fig = plt.figure(figsize=(4, 4))
        fig.suptitle(title)

        # Apply style
        if (self.__mode == '2D'):
            ax = fig.add_subplot(1,1,1)
        else:
            ax = fig.add_subplot(1,1,1, projection='3d')
            ax.xaxis.set_pane_color("white", alpha=None)
            ax.yaxis.set_pane_color("white", alpha=None)
            ax.zaxis.set_pane_color("white", alpha=None)
            # ax.view_init(50, -50)

        # Move the window 
        self.__move_figure(fig, x, y)

        return fig, ax


    def __update_plot(self, frame):
        """
        Update the plot.
        Parameters:
            - frame: MDS or WLP to choose the data
        """

        def get_data(f):
            if (f == 'MDS'): return self.__axis_MDS, self.__MDS_coords
            else: return self.__axis_WLP, self.__WLP_coords

        axis, data = get_data(frame)
        
        # Clear the previous plot
        axis.clear()
        
        if (data is not None): # data might not be initialized yet
            if (self.__mode == '2D'):
                d = self.__2Dmethod.fit_transform(data) # reduce to 2D
                t = self.__2Dmethod.fit_transform(self.__true_coords) # reduce to 2D
                axis.scatter(self.__true_coords[0],  self.__true_coords[1], c="red"  ) # Plot true coordinates
                axis.scatter(d[0], d[1], c="black") 
            else:
                axis.scatter(self.__true_coords[0],  self.__true_coords[1],  self.__true_coords[2],  c="red"  ) # Plot true coordinates
                axis.scatter(data[0], data[1], data[2],  c="black")
    

    def __plot_thread(self):

        # Initialize the plot object and enable animation
        if (self.__display_MDS): 
            self.__figure_MDS, self.__axis_MDS = self.__initialize_figure('MDS algorithm', 1500, 50 )
            ani_mds = animation.FuncAnimation(self.__figure_MDS, lambda _: self.__update_plot('MDS'), 
                                                    frames=None, interval=self.__frequency, cache_frame_data=False)

        if (self.__display_WLP): 
            self.__figure_WLP, self.__axis_WLP = self.__initialize_figure('WLP algorithm', 1500, 800)
            ani_wlp = animation.FuncAnimation(self.__figure_WLP, lambda _: self.__update_plot('WLP'), 
                                                    frames=None, interval=self.__frequency, cache_frame_data=False)
        plt.show() 


    def __init__(self, mode='2D', display_MDS=True, display_WLP=True, frequency=200, reduction_method='PCA', disable_toolbar=True):
        """
        Class to plot data.
        Parameters:
            - mode: 2D and 3D options available
            - display_MDS: show MDS data. True by default
            - display_WLP: show WLP data. True by default
            - frequency: time between each plot update        
            - reduction_method: only for 2D mode - method for dimensionality reduction
            - disable_toolbar: enable or disable plot toolbar (navigation panel)
        """
        try:
            matplotlib.use('TkAgg')
        except:
            raise SystemError("Impossible to start class with TkAgg as X-server")

        if (mode not in ['2D', '3D']): raise ValueError(f"mode parameter must be either '2D' or '3D', not {mode}.")

        # Set the class attributes
        self.__mode = mode
        self.__display_MDS = display_MDS
        self.__display_WLP = display_WLP
        self.__frequency = frequency
        self.__disable_toolbar = disable_toolbar

        self.__true_coords, self.__MDS_coords, self.__WLP_coords, self.__MDS_cov, self.__WLP_cov  \
                                                                            = None, None, None, None, None

        # Choose dimensionality reduction method
        if (self.__mode == '2D'):
            if (reduction_method == 'PCA'): self.__2Dmethod = PCA(n_components=2)
            else: raise ValueError(f"reduction_method must be PCA, not {reduction_method}")


    def start(self):
        """
        Start the plotting as a thread.
        """
        self.plot_thread = threading.Thread(target=self.__plot_thread)
        self.plot_thread.start()


    def update(self, true_coords: np.ndarray, 
                    MDS_coords: np.ndarray = None, WLP_coords : np.ndarray = None,
                    MDS_cov : np.ndarray = None, WLP_cov : np.ndarray = None):
        """
        Update the stored data for the respective plot.
        Parameters:
            - true_coords: true coordinates of points
            - MDS_coords: coordinates estimated via MDS algorithm
            - WLP_coords: corrdinates estimated via WLP algorithm
            - MDS_cov: covariance matrix associated to each point, estimated via MDS algorithm
            - WLP_cov: covariance matrix associated to each point, estimated via WLP algorithm
        """

        assert type(true_coords) == np.ndarray
        self.__true_coords = true_coords
        
        if (MDS_coords is not None): # Mean update
            if (type(MDS_coords) == np.ndarray and self.__display_MDS): self.__MDS_coords = MDS_coords
            else: raise ValueError("MDS_coords is not of type np.ndarray or MDS was not set to visible during class initialization")

        if (MDS_cov is not None):    # Covariance update
            if (type(MDS_cov) == np.ndarray and self.__display_MDS): self.__MDS_cov = MDS_cov
            else: raise ValueError("MDS_cov is not of type np.ndarray or MDS was not set to visible during class initialization")

        if (WLP_coords is not None): # Mean update
            if (type(WLP_coords) == np.ndarray and self.__display_WLP): self.__WLP_coords = WLP_coords
            else: raise ValueError("WLP_coords is not of type np.ndarray or WLP was not set to visible during class initialization")

        if (WLP_cov is not None):    # Covariance update
            if (type(MDS_cov) == np.ndarray and self.__display_WLP): self.__WLP_cov = WLP_cov
            else: raise ValueError("WLP_cov is not of type np.ndarray or WLP was not set to visible during class initialization")



# 
# Usage example
# 

# if __name__ == '__main__':
#     import time
#     # Initialize the class
#     test = Plot(mode='2D', display_MDS=True, display_WLP=True)

#     # Start the thread
#     test.start()

#     # Update data over time
#     while True:
#         data1 = np.random.uniform(low=-5, high=5, size=(3,10))
#         data2 = np.random.uniform(low=-2, high=2, size=(3,10))
#         data3 = np.random.uniform(low=-2, high=2, size=(3,10))

#         test.update(true_coords=data1, MDS_coords=data2, WLP_coords=data3)
#         time.sleep(2)