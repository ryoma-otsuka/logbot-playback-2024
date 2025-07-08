#### Setup ####
source("src/utils.R")
setup()

sec = 5
data_type_list = c("s_vedba", "ad_speed", "ad_mpr")
slope_list = c()
hist_list = c()
dens_list = c()
for (i in 1:length(data_type_list)) 
{
  data_type = data_type_list[[i]]
  if (data_type == "s_vedba")
  {
    title = "S-VeDBA"
  } else if (data_type == "ad_speed")
  {
    title = "AD-Speed"
  } else if (data_type == "ad_mpr")
  {
    title = "AD-MPR"
  }
  fname_base = sprintf("%s_%02ds", data_type, sec)
  path = sprintf("data/%s.csv", fname_base)
  print(path)
  df = read.csv(path)
  df = subset(df, audio_file_name != "Cancelled") # filtering
  print(unique(df$test_id))
  df_raw = df
  # print(df)
  
  #### Slope plot ####
  # slope plot
  df2 = prep_df2_for_slope_plot(df)
  p_slope = vis_slope_plot(df2, x_labels=c("Pre", "Post"))
  p_slope = p_slope +
    theme(panel.grid.major.x = element_blank()) +
    theme(panel.grid.minor.x = element_blank()) +
    theme(axis.title.y = element_text(margin = margin(r = 30))) +
    theme(plot.margin = unit(c(5, 15, 5, 5), "mm")) +
    NULL
  slope_list[[i]] = p_slope
  print(p_slope)
  
  #### Histogram ####
  if (data_type == "s_vedba")
  {
    title = "S-VeDBA"
    x_lim = c(-0.5, 0.5)
  } else if (data_type == "ad_speed")
  {
    title = "AD-Speed"
    x_lim = c(-2.0, 2.0)
  } else if (data_type == "ad_mpr")
  {
    title = "AD-MPR"
    x_lim = c(-0.03, 0.03)
  }
  # histogram
  p = ggplot(data=df, mapping=aes(x=diff_value)) +
    geom_histogram(biss = 10, fill = "#56B4E9", color = "transparent", alpha=0.75) +
    labs(title=title, x="Y", y="Count") +
    scale_y_continuous(breaks = scales::pretty_breaks(n = 5)) +
    coord_cartesian(xlim = x_lim) +
    theme(axis.title.y = element_text(margin = margin(r = 30))) +
    theme(plot.margin = unit(c(5, 15, 5, 5), "mm")) +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10))) +
    NULL
  print(p)
  hist_list[[i]] = p
  
  #### Density plot ####
  # density
  p = ggplot(data=df, mapping=aes(x=diff_value)) +
    geom_density(fill = "#009E73", alpha=0.2, color = "#009E73", size=0.5) +
    labs(title=title, x="Y", y="Density") +
    coord_cartesian(xlim = x_lim) +
    theme(axis.title.y = element_text(margin = margin(r = 30))) +
    theme(plot.margin = unit(c(5, 15, 5, 5), "mm")) +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10))) +
    NULL
  print(p)
  dens_list[[i]] = p
  print(sprintf("Number of zero values: %d", sum(df$diff_value == 0)))
}


p1 = slope_list[[1]]
p2 = slope_list[[2]]
p3 = slope_list[[3]]

p4 = hist_list[[1]]
p5 = hist_list[[2]]
p6 = hist_list[[3]]

p7 = dens_list[[1]]
p8 = dens_list[[2]]
p9 = dens_list[[3]]

#### Slope + Histogram ####
# Create a grid of plots
p = cowplot::plot_grid(
  p1, p2, p3, p4, p5, p6,
  labels = c("(a)", "(b)", "(c)", "(d)", "(e)", "(f)"), # Add labels to each plot
  label_size = 18,                  # Set the font size for labels
  label_x = -0.03,                  # Adjust the x position of labels
  label_y = 1.03,                   # Adjust the y position of labels
  nrow = 2, ncol = 3,               # Set the number of rows and columns in the grid
  align = "hv",                     # Align plots horizontally and vertically
  rel_heights = c(1, 1)             # Set the relative heights for each row
)

# Apply theme adjustments
p = p + theme(
  plot.margin = margin(7, 5, 5, 5, unit = "mm"),  # Adjust margins around the plot grid
  plot.background = element_rect(fill = "white", color = NA) # Set the background color to white
)

print(p)

#### Histogram + Density ####
# Create a grid of plots
p = cowplot::plot_grid(
  p4, p5, p6, p7, p8, p9,
  labels = c("(a)", "(b)", "(c)", "(d)", "(e)", "(f)"),  # Add labels to each plot
  label_size = 18,                  # Set the font size for labels
  label_x = -0.03,                  # Adjust the x position of labels
  label_y = 1.03,                   # Adjust the y position of labels
  nrow = 2, ncol = 3,               # Set the number of rows and columns in the grid
  align = "hv",                     # Align plots horizontally and vertically
  rel_heights = c(1, 1)             # Set the relative heights for each row
)

# Apply theme adjustments
p = p + theme(
  plot.margin = margin(7, 5, 5, 5, unit = "mm"),  # Adjust margins around the plot grid
  plot.background = element_rect(fill = "white", color = NA) # Set the background color to white
)

print(p)

#### Slope + Histogram + Density ####
# Create a grid of plots
p = cowplot::plot_grid(
  p1, p2, p3, p4, p5, p6, p7, p8, p9,
  labels = c("(a)", "(b)", "(c)", "(d)", "(e)", "(f)", "(g)", "(h)", "(i)"),  # Add labels to each plot
  label_size = 18,                    # Set the font size for labels
  label_x = -0.03,                    # Adjust the x position of labels
  label_y = 1.03,                     # Adjust the y position of labels
  nrow = 3, ncol = 3,                 # Set the number of rows and columns in the grid
  align = "hv",                       # Align plots horizontally and vertically
  rel_heights = c(1, 1, 1)            # Set the relative heights for each row
)

# Apply theme adjustments
p = p + theme(
  plot.margin = margin(7, 5, 5, 5, unit = "mm"),  # Adjust margins around the plot grid
  plot.background = element_rect(fill = "white", color = NA) # Set the background color to white
)

print(p)

remove(list = ls())
