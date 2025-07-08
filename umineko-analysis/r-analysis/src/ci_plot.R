# https://github.com/google/CausalImpact/blob/master/R/impact_plot.R

library(assertthat)

GetPeriodIndices <- function(period, times) {
  # Computes indices belonging to a period in data.
  #
  # Args:
  #   period:  two-element vector specifying start and end of a period, having
  #            the same data type as `times. The range from `period[1]` to
  #            `period[2]` must have an intersect with `times`.
  #   times:   vector of time points; can be of integer or of POSIXct type.
  #
  # Returns:
  #   A two-element vector with the indices of the period start and end within
  #   `times`.
  
  # Check input
  assert_that(length(period) == 2)
  assert_that(!anyNA(times))
  assert_that(identical(class(period), class(times)) ||
                (is.numeric(period) && is.numeric(times)))
  # Check if period boundaries are in the right order, and if `period` has an
  # overlap with `times`.
  assert_that(period[1] <= period[2])
  assert_that(period[1] <= tail(times, 1), period[2] >= times[1])
  
  # Look up values of start and end of period in `times`; also works if the
  # period start and end time are not exactly present in the time series.
  indices <- seq_along(times)
  is.period <- (period[1] <= times) & (times <= period[2])
  # Make sure the period does match any time points.
  assert_that(any(is.period),
              msg = "The period must cover at least one data point")
  period.indices <- range(indices[is.period])
  return(period.indices)
}

CreatePeriodMarkers <- function(pre.period, post.period, times) {
  # Creates a vector of period markers to display.
  #
  # Args:
  #   pre.period:  vector of 2 time points that define the pre-period.
  #   post.period: vector of 2 time points that define the post-period.
  #   times:       vector of time points.
  #
  # Returns:
  #   Vector of period markers that should be displayed, generally depicting the
  #   first and last time points of pre- and post-period. The start of the pre-
  #   period is not shown if it coincides with the first time point of the time
  #   series; similarly, the last time point of the post-period is not shown if
  #   it coincides with the last time point of the series. If there is no gap
  #   between pre- and post-period, the start marker of the post-period is
  #   omitted.
  
  pre.period.indices <- GetPeriodIndices(pre.period, times)
  post.period.indices <- GetPeriodIndices(post.period, times)
  markers <- NULL
  if (pre.period.indices[1] > 1) {
    markers <- c(markers, times[pre.period.indices[1]])
  }
  markers <- c(markers, times[pre.period.indices[2]])
  if (pre.period.indices[2] < post.period.indices[1] - 1) {
    markers <- c(markers, times[post.period.indices[1]])
  }
  if (post.period.indices[2] < length(times)) {
    markers <- c(markers, times[post.period.indices[2]])
  }
  markers <- as.numeric(markers)
  return(markers)
}


plot_causal_impact = function(
    impact, 
    title,
    metrics = c("original", "pointwise", "cumulative")
)
{
  
  p = plot(impact)
  plot_data = ggplot_build(p)
  data = plot_data$plot$data
  
  # Initialize plot
  q <- ggplot(data, aes(x = time)) + theme_bw(base_size = 15)
  q <- q + xlab("") + ylab("")
  if (length(metrics) > 1) {
    q <- q + facet_grid(metric ~ ., scales = "free_y")
  }
  
  # Add prediction intervals
  q <- q + geom_ribbon(aes(ymin = lower, ymax = upper),
                       data, 
                       fill = "#1E88E5", 
                       alpha=0.25)
  
  # Add pre-period markers
  xintercept <- CreatePeriodMarkers(impact$model$pre.period,
                                    impact$model$post.period,
                                    time(impact$series))
  q <- q + geom_vline(xintercept = xintercept,
                      colour = "darkgrey", 
                      size = 1.0, 
                      linetype = "dashed")
  
  # Add zero line to pointwise and cumulative plot
  q <- q + geom_line(aes(y = baseline),
                     colour = "darkgrey", 
                     size = 1.0, 
                     linetype = "solid",
                     na.rm = TRUE)
  
  # Add point predictions
  q <- q + geom_line(aes(y = mean), 
                     data,
                     size = 1.0, 
                     colour = "#1E88E5", 
                     linetype = "dashed",
                     na.rm = TRUE)
  
  # Add observed data
  q <- q + geom_line(aes(y = response), size = 0.6, na.rm = TRUE)
  
  q <- q + labs(title=title)
  q <- q + theme(plot.title = element_text(size = 14, face = "bold", hjust = 0.5, margin = margin(b = 10)))
  
  
  return(q)
}