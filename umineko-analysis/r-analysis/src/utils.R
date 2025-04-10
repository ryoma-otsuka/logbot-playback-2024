#### Libraries ####
library(dplyr)
library(rstan)
library(rethinking)
library(loo)
library(tidybayes)
library(ggplot2)
library(svglite)
library(extrafont)
extrafont::loadfonts(device = "win")

#### Setup ####
setup = function(use_stan=TRUE, is_rmd=FALSE)
{
  # visualization setup
  ggplot() +
    theme_set(
      theme_bw() +
        theme(axis.title.x = element_text(size = 14)) +
        theme(axis.text = element_text(size = 12)) + 
        theme(axis.title.y = element_text(
          size = 14, margin = margin(t = 0, r = 10, b = 0, l = 0))
        ) + 
        theme(legend.title = element_text(size = 12)) + 
        theme(legend.text = element_text(size = 10)) +
        theme(axis.ticks.x = element_line(linewidth = 0.7)) +
        theme(axis.ticks.y = element_line(linewidth = 0.7)) +
        theme(plot.title = element_text(size = 16, face = "bold")) +
        theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10))) +
        theme(panel.border = element_rect(linewidth = 1.2, color = "black")) +
        theme(strip.background = element_rect(linewidth = 1.2, color = "black")) +
        theme(strip.text = element_text(size = 12)) +
        theme(panel.grid.major.x = element_blank()) +
        theme(panel.grid.minor.x = element_blank()) +
        theme(panel.grid.major.y = element_line()) +
        theme(panel.grid.minor.y = element_line()) +
        theme(plot.margin = unit(c(5, 5, 5, 5), "mm")) + 
        theme(text = element_text(family = "Segoe UI")) +
        theme(axis.text = element_text(color = "black"))
    )
  
  if (use_stan)
  {
    # for speeding up MCMC sampling
    rstan_options(auto_write = TRUE)
    # options(mc.cores = parallel::detectCores())
  }
  
  if (is_rmd)
  {
    library(knitr)
    # Grobal options
    opts_knit$set(root.dir = "r-analysis/")
    knitr::opts_chunk$set(
      echo = TRUE,
      cache = TRUE,
      prompt = FALSE,
      tidy = TRUE,
      comment = NA,
      message = TRUE,
      warning = TRUE
    )
  }
}


#### MCMC Convergence diagnosis ####
check_rhat = function(fit, threshold_list=c(1.010, 1.005))
{
  for (i in 1:length(threshold_list))
  {
    threshold = threshold_list[[i]]
    is_ok = all(summary(fit)$summary[, 'Rhat'] < threshold, na.rm = T)
    print(sprintf("all rhat value < threshold: %.3f? %s", threshold, is_ok))
  }
  return (is_ok)
}

check_divergent_transitions = function(fit)
{
  sampler_params = get_sampler_params(fit, inc_warmup=FALSE)
  divergent_count = sapply(sampler_params, function(x) sum(x[, "divergent__"]))
  total_divergent = sum(divergent_count)
  print(sprintf("N of divergent transition: %s", total_divergent))
  
  
  
  return (total_divergent)
}

vis_traceplot = function(
    fit, 
    parameters, 
    ncol=5, 
    inc_warmup=FALSE,
    separate_chains=TRUE
    )
{
  
  # color set
  # colors = c("#d62728", "#2ca02c", "#ff7f0e", "#1f77b4")
  colors = c('#F0E442', '#E69F00', '#56B4E9', '#009E73')
  # colors = c('#CC79A7', '#E69F00', '#56B4E9', '#009E73')
  # colors = c('#fde725', '#35b779',  '#31688e', '#440154')
  # colors = c('#ffd700', '#fa8775',  '#cd34b5', '#0000ff')
  
  # base
  p_set = theme_bw(base_size=12) +
    theme(axis.text=element_text(color="black"))
  
  # trace plot
  p_traceplot = rstan::traceplot(
    fit, pars=parameters,
    ncol=ncol, inc_warmup=inc_warmup
  ) +
    scale_color_manual(values=colors) +
    p_set + 
    NULL
  print(p_traceplot)
  
  return (p_traceplot)
}


vis_histogram = function(
    fit, 
    parameters, 
    ncol=5, 
    inc_warmup=FALSE,
    separate_chains=TRUE
)
{
  
  # color set
  # colors = c("#d62728", "#2ca02c", "#ff7f0e", "#1f77b4")
  colors = c('#F0E442', '#E69F00', '#56B4E9', '#009E73')
  
  # base
  p_set = theme_bw(base_size=12) +
    theme(axis.text=element_text(color="black"))
  
  # histogram
  p_histogram = rstan::stan_hist(
    fit, pars=parameters, ncol=ncol, 
    fill="#009E73", col="transparent", alpha=0.7, bins=30
  ) + 
    geom_vline(xintercept = c(0), linetype = "dashed") +
    labs(y="Count") + 
    p_set +
    NULL
  print(p_histogram)
  
  return (p_histogram)
}


vis_density = function(
    fit, 
    parameters, 
    ncol=5, 
    inc_warmup=FALSE,
    separate_chains=TRUE,
    title=0
)
{
  
  # color set
  # colors = c("#d62728", "#2ca02c", "#ff7f0e", "#1f77b4")
  colors = c('#F0E442', '#E69F00', '#56B4E9', '#009E73')
  
  # base
  p_set = theme_bw(base_size=12) +
    theme(axis.text=element_text(color="black")) +
    theme(panel.grid.major.x = element_line()) +
    theme(panel.grid.minor.x = element_line()) +
    theme(panel.grid.major.y = element_line()) +
    theme(panel.grid.minor.y = element_line()) +
    NULL
  
  # kernel density plot
  p_density = rstan::stan_dens(
    fit, pars=parameters, ncol=ncol, 
    separate_chains=separate_chains, alpha=0.5
  ) +
    geom_vline(xintercept = c(0), linetype = "dashed") +
    # scale_color_manual(values=colors) + 
    scale_fill_manual(values=colors) + 
    p_set + 
    NULL
  
  if (any(grepl("u", parameters)))
  {
    parameter_labels = c(
      `u[1]` = "u[1]: LBP00", `u[2]` = "u[2]: LBP01", `u[3]` = "u[3]: LBP03", `u[4]` = "u[4]: LBP05", 
      `u[5]` = "u[5]: LBP06", `u[6]` = "u[6]: LBP07", `u[7]` = "u[7]: LBP08", `u[8]` = "u[8]: LBP09"
    )
    p_density = p_density + 
      facet_wrap(
        ~ parameter, 
        labeller = labeller(parameter = parameter_labels), 
        scales = "free_x",
        ncol = ncol)
  }
  
  if (title == 0)
  {
    p_density = p_density + labs(y="Density")
  } else
  {
    p_density = p_density + labs(title=title, y="Density") +
      theme(plot.title = element_text(size = 16, face = "bold")) +
      theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10)))
  }
  
  print(p_density)
  
  return (p_density)
}


vis_trace_hist_density = function(
    fit, 
    parameters, 
    ncol=5, 
    inc_warmup=FALSE,
    separate_chains=TRUE
)
{
  
  p_traceplot = vis_traceplot(fit, parameters, ncol, inc_warmup, separate_chains)
  p_histogram = vis_histogram(fit, parameters, ncol, inc_warmup, separate_chains)
  p_density = vis_density(fit, parameters, ncol, inc_warmup, separate_chains)
  plot_list = c()
  plot_list[[1]] = p_traceplot
  plot_list[[2]] = p_histogram
  plot_list[[3]] = p_density
  return (plot_list)
}

#### Posterior distributions ####
vis_slope_posteriors = function(fit, data_type)
{

  if (data_type == "s_vedba")
  {
    title = "S-VeDBA"
    # xlim_lower = -0.35
    # xlim_upper = 0.35
    factor = 1.1
  } else if (data_type == "ad_speed")
  {
    title = "AD-Speed"
    # xlim_lower = -2.0 
    # xlim_upper = 2.0
    factor = 1.0
  } else if (data_type == "ad_mpr")
  {
    title = "AD-MPR"
    # xlim_lower = -0.02
    # xlim_upper = 0.02
    factor = 1.0
  } else 
  {
    title = "Simulation"
    factor = 1.5
  }
  
  # n of beta dims
  beta_samples = rstan::extract(fit)$beta
  beta_dim = ncol(beta_samples)
  print(sprintf("beta_dim: %s", beta_dim))
  
  max_val = max(beta_samples)
  min_val = min(beta_samples)
  extreme_val = ifelse(abs(max_val) > abs(min_val), max_val, min_val)
  axis_limit = abs(extreme_val) * factor
  xlim_lower = -axis_limit
  xlim_upper = axis_limit 
  
  # plot posteriors
  p = fit %>%
    spread_draws(beta[K]) %>%
    ggplot(aes(y = factor(K, rev(1:beta_dim)), x = beta)) +
    # geom_vline(xintercept = c(0), linetype = "dashed", alpha=0.1) +
    # posterior distribution + 89%, 97% CI
    stat_halfeye(fill="#56B4E9", .width = c(0.89, 0.97), slab_alpha=0.7) + 
    scale_y_discrete(
      labels = sapply(rev(seq_len(beta_dim)), function(i) bquote(beta[.(i)]))
    ) + 
    labs(
      title = sprintf("%s", title),
      x = "Value",
      y = "Parameter"
    ) + 
    theme(panel.grid.major.y = element_blank()) +
    theme(panel.grid.minor.y = element_blank()) +
    theme(panel.grid.major.x = element_line()) +
    theme(panel.grid.minor.x = element_line()) +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10)))
  
  # set xlim
  p = p + xlim(xlim_lower, xlim_upper)
  
  # Add text | Posterior median + 97 % CI
  posterior_summary = fit %>%
    spread_draws(beta[K]) %>%
    group_by(K) %>%
    summarise(
      median = median(beta),
      lower = quantile(beta, 0.015),
      upper = quantile(beta, 0.985)
    )
  
  y_offset = 0.20
  x_offset = 0.00
  
  p = p + 
    geom_text(
      data = posterior_summary,
      aes(
        factor(K, rev(1:beta_dim)) %>% as.numeric() + y_offset,
        x = median + x_offset,
        label = sprintf("%.4f [%.4f, %.4f]", median, lower, upper)
      ),
      color = "#000000",
      hjust = 0.5,  # centering
      lineheight = 0.8  # line space
    )
  
  # print(p)
  return (p)
}


vis_randam_intercept_posteriors = function(fit, data_type)
{
  
  if (data_type == "s_vedba")
  {
    title = "S-VeDBA"
    xlim_lower = -0.5
    xlim_upper = 0.5
    factor = 1.1
  } else if (data_type == "ad_speed")
  {
    title = "AD-Speed"
    xlim_lower = -2.5
    xlim_upper = 2.5
    factor = 1.0
  } else if (data_type == "ad_mpr")
  {
    title = "AD-MPR"
    xlim_lower = -0.025
    xlim_upper = 0.025
    # factor = 1.0
  } else
  {
    title = "Simulation"
    xlim_lower = -0.02
    xlim_upper = 0.02
    factor = 1.5
  }

  # Extract relevant parameters: α, u[1:8], sigma_u
  alpha_samples = rstan::extract(fit)$alpha
  u_samples = rstan::extract(fit)$u
  sigma_u_samples = rstan::extract(fit)$sigma_u
  
  # Convert u_samples into a data frame, assuming u_samples is a matrix
  u_samples_df <- as.data.frame(u_samples)
  colnames(u_samples_df) <- paste0("u[", 1:ncol(u_samples_df), "]")
  
  # Combine alpha, u, and sigma_u into one data frame
  posterior_samples <- data.frame(
    alpha = alpha_samples,
    sigma_u = sigma_u_samples
  )
  posterior_samples <- cbind(posterior_samples, u_samples_df)
  
  # Create a long format data frame for plotting
  posterior_long = posterior_samples %>%
    as_tibble() %>%
    tidyr::pivot_longer(cols = everything(), names_to = "parameter", values_to = "value")
  
  # Create a parameter order for display
  parameter_levels <- rev(c("alpha", paste0("u[", 1:8, "]"), "sigma_u"))
  
  # max_val = max(posterior_samples)
  # min_val = min(posterior_samples)
  # extreme_val = ifelse(abs(max_val) > abs(min_val), max_val, min_val)
  # axis_limit = abs(extreme_val) * factor
  # xlim_lower = -axis_limit
  # xlim_upper = axis_limit

  # plot posteriors
  p = ggplot(data = posterior_long, 
             aes(x = value, y = factor(parameter, levels = parameter_levels))) +
    stat_halfeye(fill="#009E73", .width = c(0.89, 0.97), slab_alpha=0.5) + 
    scale_y_discrete(labels = parameter_levels) + 
    labs(
      title = sprintf("%s", title),
      x = "Value",
      y = "Parameter"
    ) + 
    theme(panel.grid.major.y = element_blank()) +
    theme(panel.grid.minor.y = element_blank()) +
    theme(panel.grid.major.x = element_line(colour = "grey80")) +
    theme(panel.grid.minor.x = element_line(colour = "grey90")) +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10)))
  
  # set xlim
  p = p + xlim(xlim_lower, xlim_upper)
  # Calculate summary statistics for plotting
  posterior_summary = posterior_long %>%
    group_by(parameter) %>%
    summarise(
      median = median(value),
      lower = quantile(value, 0.015),
      upper = quantile(value, 0.985)
    )
  
  y_offset = 0.20
  x_offset = 0.00
  # print(head(posterior_summary))
  p = p +
    geom_text(
      data = posterior_summary,
      aes(
        x = median + x_offset,
        y = factor(parameter, levels = parameter_levels) %>% as.numeric() + y_offset,
        label = sprintf("%.4f [%.4f, %.4f]", median, lower, upper)
      ),
      color = "#000000",
      hjust = 0.5,  # centering
      lineheight = 0.8  # line space
    )
  
  # print(p)
  return (p)
}

#### Posterior predictive checks####
vis_posterior_predictive_check = function(fit, Y, title, I=50)
{
  # base plot
  p3 = ggplot(mapping=aes(x=Y)) +
    geom_density(size=0.75, trim=TRUE) +
    labs(title = title, x = "Y", y = "Density") +
    theme(panel.grid.minor.x = element_line(), 
          panel.grid.major.x = element_line()) +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10))) +
    NULL

  # just plot the first 100 simulated data
  ms = rstan::extract(fit)
  # print(str(ms))
  Y_pred = rep(NA, ncol(ms$Y_pred))
  df_pred = data.frame(Y_pred = Y_pred)
  
  # Randomly extract I columns and plot their posterior prediction density
  set.seed(1234)
  columns_to_plot = sample(1:ncol(ms$Y_pred), I) # a vector for 
  
  PRED_COLOR = "#56B4E9"
  DATA_COLOR = "#333333"
  
  # PRED_COLOR = "#888888"
  # DATA_COLOR = "#56B4E9"

  for (i in columns_to_plot) {
    df_pred$Y_pred = ms$Y_pred[i, ]
    p3 = p3 + 
      geom_density(
        data=df_pred, mapping=aes(x=Y_pred), 
        color=PRED_COLOR,
        size=0.3, trim=TRUE
      )
  }
  p3 = p3 + geom_density(
    mapping=aes(x=Y), color=DATA_COLOR, size=1.0, trim=TRUE
  )
  
  plot_build = ggplot_build(p3)
  y_lim = plot_build$layout$panel_params[[1]]$y.range
  y_lim[[2]]  = y_lim[[2]] * 1.1
  p3 = p3 + coord_cartesian(ylim=y_lim)
  
  print(p3)
  
  return (p3)
}


#### Slope plot ####
vis_slope_plot = function(df, x_labels, y_lim) {
  
  if (data_type == "s_vedba")
  {
    title = "S-VeDBA"
    y_lim = c(-0.05, 1.83)
  } else if (data_type == "ad_speed")
  {
    title = "AD-Speed"
    y_lim = c(-0.4, 3.5)
  } else if (data_type == "ad_mpr")
  {
    title = "AD-MPR"
    y_lim = c(-0.005, 0.033)
  }
  
  # dodge = position_dodge(width = 1.2)
  
  # Define nudging based on condition
  df$nudge_values = ifelse(df$Condition == "pre_value", 0.2, -0.2)
  # Adjust the x-axis positions based on condition and nudge values
  df$x_position = as.numeric(df$Condition) + df$nudge_values  # Apply the nudge to the x-axis positions
  
  color_mapping = c("Even"="grey", "Negative"="#1E88E5", "Positive"="#D81B60")
  p = ggplot(data = df, aes(x=Condition, y=Values)) +
    
    # violin plot in grey, no outline
    geom_violin(
      alpha=0.5,
      fill="grey70",
      width=0.32,
      color=NA,
      show.legend=FALSE,
      trim = FALSE,
      bounds = c(0, Inf)
    ) +
    # data points
    geom_point(
      aes(col=Difference, group=ID),
      color="#333333", 
      size=2.5, 
      shape=1, 
      stroke=0.7,
      position = position_nudge(x = df$nudge_values)
    ) +
    # slope
    geom_line(
      aes(x=x_position, col=Difference, group=ID), 
      linewidth=0.7, 
      alpha=0.7,
    ) +
    # color
    scale_color_manual(values = color_mapping) +
    scale_fill_manual(values = color_mapping) +
    # show box plots
    geom_boxplot(
      alpha = 0.2,
      fill = NA, 
      outlier.shape = NA, 
      notch = FALSE,
      width = 0.1, 
      size = 0.7,   # box line
      fatten = 1.5, # median line 
    ) + 
    
    # settings
    scale_x_discrete(labels = x_labels) +
    # scale_y_continuous(breaks = seq(from=0, to=1400, by=200)) +
    coord_cartesian(xlim = c(1.3, 1.7), ylim = y_lim) +
    theme(legend.position = "none") +
    theme(axis.title.x = element_blank()) +
    labs(title = sprintf("%s", title), y="Mean value") +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10))) +
    NULL
  
  return(p)
}

prep_df2_for_slope_plot = function(df) {
  Diff = df$diff_value
  diff_str = c(rep(NA, length(Diff)))
  for (i in 1:length(Diff)) {
    if (Diff[i] > 0) {
      diff_str[i] = "Positive"
    }
    else if (Diff[i] == 0) {
      diff_str[i] = "Even"
    }
    else diff_str[i] = "Negative"
  }
  # head(diff_str)
  df1 = df[, c(4:5)]
  # print(df1)
  df2 = df1 %>% 
    tidyr::gather(key = Condition, value = Values) %>% 
    mutate(ID = rep(c(1:length(df[,1])), 2)) %>%
    mutate(Difference = rep(diff_str, 2))
  # print(df2)
  
  df2$Condition = factor(df2$Condition, levels = c("pre_value", "post_value"))
  
  return (df2)
}

