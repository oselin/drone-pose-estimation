#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from sklearn.decomposition import PCA
import threading
import warnings
import matplotlib
from matplotlib.patches import Ellipse
warnings.simplefilter("ignore", UserWarning)


class Plot():

    def __move_figure(self, f, x, y):
        """
        Move figure's upper left corner to pixel [x, y].
        """
        backend = matplotlib.get_backend()
        if (backend == 'TkAgg'):
            f.canvas.manager.window.wm_geometry(f'+{x}+{y}')
        elif (backend == 'WXAgg'):
            f.canvas.manager.window.SetPosition((x, y))
        else:
            f.canvas.manager.window.move(x, y)  # works for QT and GTK

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
        if (self.__disable_toolbar):
            matplotlib.rcParams['toolbar'] = 'None'
        # Initialize the figure
        fig = plt.figure(figsize=(4, 4))
        fig.suptitle(title)

        # Apply style
        if (self.__mode == '2D'):
            ax = fig.add_subplot(1, 1, 1)
        else:
            ax = fig.add_subplot(1, 1, 1, projection='3d')
            ax.xaxis.set_pane_color("white", alpha=None)
            ax.yaxis.set_pane_color("white", alpha=None)
            ax.zaxis.set_pane_color("white", alpha=None)
            # ax.view_init(50, -50)

        # Move the window
        self.__move_figure(fig, x, y)

        return fig, ax

    def __get_data(self, frame):
        """
        Return axis and coordinates for the given frame.
        Parameters:
            - frame: 'MDS' or 'LSM'
        """
        if (frame == 'MDS'):
            return self.__axis_MDS, self.__MDS_coords, self.__MDS_cov
        else:
            return self.__axis_LSM, self.__LSM_coords, self.__LSM_cov

    def __data_to_2D(self, data):
        """
        Convert 3D data in 2D, by using different techniques.
        Methods supported: PCA, xy
        Parameters:
            - data: 3D data to be converted, shape=[3, n]
        Return:
            - 2D data
        """
        if (self.__reduction_method == 'PCA'):
            pca = PCA(n_components=2)
            return pca.fit_transform(data)

        elif (self.__reduction_method == 'xy'):
            return data[:2]

    def __update_plot(self, frame):
        """
        Update the plot.
        Parameters:
            - frame: MDS or LSM to choose the data
        """
        axis, data, _ = self.__get_data(frame)

        # Clear the previous plot
        axis.clear()

        if (data is not None):  # data might not be initialized yet
            if (self.__mode == '2D'):
                t = self.__data_to_2D(self.__true_coords)  # reduce to 2D
                d = self.__data_to_2D(data)               # reduce to 2D

                # Plot true coordinates
                axis.scatter(t[0], t[1], c="red")
                # Plot estimated coordinates
                axis.scatter(d[0], d[1], c="black", s=5)

                x_ref = self.__true_coords[0, 1]
                y_ref = self.__true_coords[1, 1]
                axis.set_xlim([x_ref-15, x_ref+15])  # ([-2,2])
                axis.set_ylim([y_ref-15, y_ref+15])  # ([-2,2])

                if (self.__display_cov):
                    self.__draw_covariance(frame=frame, confidence=0.95)
            else:
                # Plot true coordinates
                axis.scatter(
                    self.__true_coords[0],  self.__true_coords[1],  self.__true_coords[2],  c="red")
                # Plot estimated coordinates
                axis.scatter(data[0], data[1], data[2],  c="black")

                axis.set_xlim([-5, 15])  # ([-2,2])
                axis.set_ylim([-5, 15])  # ([-2,2])
                axis.set_zlim([-5, 15])  # ([-2,2])

    def __draw_covariance(self, frame: str, confidence=0.95):
        """
        Draw the covariance ellipse. Note: it works only for 2D plots.
        Parameters:
            - frame: either MDS or LSM
            - confidence: confidence level to be plotted
        """

        if (self.__mode != '2D'):
            return

        # Get points and axis for the given frame
        axis, points, cov_matrix = self.__get_data(frame=frame)

        # Covariance might not be initialized yet
        if (cov_matrix is None):
            return

        # Project to 2D
        points = self.__data_to_2D(points)

        for i in range(cov_matrix.shape[1]):

            # Get the i-th covariance matrix
            cov_i = cov_matrix[:, i].reshape(3, 3)

            # Get the i-th point
            point_i = points[:, i]

            # 1 - Decompose the covariance matrix
            eigenvalues, eigenvectors = np.linalg.eigh(cov_i)

            # 2- Determine major and minor axes
            confidence_level = np.sqrt(
                eigenvalues) * np.sqrt(-2 * np.log(1 - confidence))
            major_axis_length = 2 * confidence_level[-1]
            minor_axis_length = 2 * confidence_level[-2]

            # 3 - Plot the ellipse
            ellipse = Ellipse(xy=point_i, width=major_axis_length,
                              height=minor_axis_length, edgecolor='b', fc='None', lw=2)

            axis.add_patch(ellipse)

    def __plot_thread(self):

        # Initialize the plot object and enable animation
        if (self.__display_MDS):
            self.__figure_MDS, self.__axis_MDS = self.__initialize_figure(
                'MDS algorithm', 1500, 50)
            ani_mds = animation.FuncAnimation(self.__figure_MDS, lambda _: self.__update_plot('MDS'),
                                              frames=None, interval=self.__frequency, cache_frame_data=False)

        if (self.__display_LSM):
            self.__figure_LSM, self.__axis_LSM = self.__initialize_figure(
                'LSM algorithm', 1500, 800)
            ani_LSM = animation.FuncAnimation(self.__figure_LSM, lambda _: self.__update_plot('LSM'),
                                              frames=None, interval=self.__frequency, cache_frame_data=False)
        plt.show()

    def __init__(self, mode='2D', display_MDS=True, display_LSM=True,
                 frequency=200, reduction_method='PCA', disable_toolbar=True,
                 display_covariance=False):
        """
        Class to plot data.
        Parameters:
            - mode: 2D and 3D options available
            - display_MDS: show MDS data. True by default
            - display_LSM: show LSM data. True by default
            - display_covariance: plot covariance ellipse. False by default
            - frequency: time between each plot update        
            - reduction_method: only for 2D mode - method for dimensionality reduction
            - disable_toolbar: enable or disable plot toolbar (navigation panel)
        """
        try:
            matplotlib.use('TkAgg')
        except:
            raise SystemError(
                "Impossible to start class with TkAgg as X-server")

        # Verify consistency of parameters
        if (mode not in ['2D', '3D']):
            raise ValueError(
                f"mode parameter must be either '2D' or '3D', not {mode}.")

        if (mode == '3D' and display_covariance):
            raise ValueError("Covariance can be plotted only in 2D, not 3D.")

        if (not (display_MDS or display_LSM) and display_covariance):
            raise ValueError(
                "At least one method must be plotted to plot covariance.")

        if (not frequency):
            raise ValueError("Frequency value must be greater than zero.")

        if (reduction_method not in ['PCA', 'xy']):
            raise ValueError(
                f"reduction_method must be either 'PCA' or 'xy', not {reduction_method}.")

        # Set the class attributes
        self.__mode = mode
        self.__display_MDS = display_MDS
        self.__display_LSM = display_LSM
        self.__frequency = frequency
        self.__disable_toolbar = disable_toolbar
        self.__display_cov = display_covariance
        self.__reduction_method = reduction_method

        self.__true_coords, self.__MDS_coords, self.__LSM_coords, self.__MDS_cov, self.__LSM_cov  \
            = None, None, None, None, None

    def start(self):
        """
        Start the plotting as a thread.
        """
        self.plot_thread = threading.Thread(target=self.__plot_thread)
        self.plot_thread.start()

    def update(self, true_coords: np.ndarray,
               MDS_coords: np.ndarray = None, LSM_coords: np.ndarray = None,
               MDS_cov: np.ndarray = None, LSM_cov: np.ndarray = None):
        """
        Update the stored data for the respective plot.
        Parameters:
            - true_coords: true coordinates of points
            - MDS_coords: coordinates estimated via MDS algorithm
            - LSM_coords: corrdinates estimated via LSM algorithm
            - MDS_cov: covariance matrix associated to each point, estimated via MDS algorithm
            - LSM_cov: covariance matrix associated to each point, estimated via LSM algorithm
        """

        if (true_coords is not None):   # True coordinates update
            if (type(true_coords) == np.ndarray):
                self.__true_coords = true_coords
            else:
                raise ValueError(
                    f"true_coords is of type {type(true_coords)}, but it must be either 'None' or 'np.ndarray'")

        if (MDS_coords is not None):    # Mean update
            if (type(MDS_coords) == np.ndarray and self.__display_MDS):
                self.__MDS_coords = MDS_coords
            else:
                raise ValueError(
                    "MDS_coords is not of type np.ndarray or MDS was not set to visible during class initialization")

        if (MDS_cov is not None):       # Covariance update
            if (type(MDS_cov) == np.ndarray and self.__display_MDS and self.__display_cov):
                self.__MDS_cov = MDS_cov
            else:
                raise ValueError(
                    "MDS_cov is not of type np.ndarray or MDS was not set to visible during class initialization")

        if (LSM_coords is not None):    # Mean update
            if (type(LSM_coords) == np.ndarray and self.__display_LSM):
                self.__LSM_coords = LSM_coords
            else:
                raise ValueError(
                    "LSM_coords is not of type np.ndarray or LSM was not set to visible during class initialization")

        if (LSM_cov is not None):       # Covariance update
            if (type(MDS_cov) == np.ndarray and self.__display_LSM and self.__display_cov):
                self.__LSM_cov = LSM_cov
            else:
                raise ValueError(
                    "LSM_cov is not of type np.ndarray or LSM was not set to visible during class initialization")


#
# Usage example
#

# if __name__ == '__main__':
#     import time
#     # Initialize the class
#     test = Plot(mode='2D', display_MDS=True, display_LSM=True, )

#     # Start the thread
#     test.start()

#     # Update data over time
#     while True:
#         data1 = np.random.uniform(low=-5, high=5, size=(3,10))
#         data2 = np.random.uniform(low=-2, high=2, size=(3,10))
#         data3 = np.random.uniform(low=-2, high=2, size=(3,10))

#         test.update(true_coords=data1, MDS_coords=data2, LSM_coords=data3, MDS_cov=None, LSM_cov=None)
#         time.sleep(2)
