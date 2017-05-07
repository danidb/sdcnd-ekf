# SDCND Extended Kalman Filter

This project is part of the Udacity self-driving car engineer nanodegree.  
Some template code was  provided  by Udacity. Here, apart from minor modifications, the overall architecture of the template was retained for the sake of consistency and ease of evaluation.  
The initial state  covariance matrix $P$ was determined by running the algorithm with an arbitrary (high variance) initial state and observing the  covariance  of the output. See `viz.R`. This facilitates convergence for future runs of the algorithm. All non-diagonal entries are assumed to  be zero. 


## Results
The Extended Kalman Filter is applied to the sample data provided by Udacity, **note that the second "new" set of sample data is used here.** The results are plotted below (see `viz.R`). 

![EKF Results.](sample-data-output.png)

The RMSE values, shown on the figure, are within within the accepted range.


