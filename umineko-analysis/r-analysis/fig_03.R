source("src/utils.R")
setup()

#### Configuration ####
sec = 5
# sec = 10

data_type_list = c("s_vedba", "ad_speed", "ad_mpr")

# model_name = "lm"
model_name = "lmm"
# model_name = "lmm_st2.2"

slope_list = list()
hist_list = list()
pos_list = list()

for (i in 1:length(data_type_list))
{
  data_type = data_type_list[[i]]
  
  #### Data ####
  fname_base = sprintf("%s_%02ds", data_type, sec)
  path = sprintf("data/mean/%s.csv", fname_base)
  print(path)
  df = read.csv(path)
  df = subset(df, audio_file_name != "Cancelled") # filtering
  Y = df$diff_value
  
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
 
  #### Histogram ####
  # histogram
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
  p_hist = ggplot(data=df, mapping=aes(x=diff_value)) +
    geom_histogram(bins = 50, fill = "#009E73", color = "transparent", alpha=0.5) +
    labs(title=title, x="Y (diff. value)", y="Count") +
    scale_y_continuous(breaks = scales::pretty_breaks(n = 5)) +
    coord_cartesian(xlim = x_lim) +
    theme(axis.title.y = element_text(margin = margin(r = 30))) +
    theme(plot.margin = unit(c(5, 15, 5, 5), "mm")) +
    theme(plot.title = element_text(hjust = 0.5, margin = margin(b = 10))) +
    NULL
  hist_list[[i]] = p_hist
  
  #### Load LMM model ####
  fname_base = sprintf("%s_%s_%02ds", model_name, data_type, sec)
  model_output_file = sprintf("output/model-output/%s.RData", fname_base)
  fit = readRDS(file = model_output_file)
  # print(fit)
  
  #### Posterior distribution ####
  # posterior distribution
  pos = vis_slope_posteriors(fit, data_type=data_type)
  pos = pos +
    theme(axis.title.y = element_text(margin = margin(r = 30))) +
    theme(plot.margin = unit(c(5, 15, 5, 5), "mm")) +
    NULL
  pos_list[[i]] = pos
}

p1 = slope_list[[1]]
p2 = slope_list[[2]]
p3 = slope_list[[3]]

p4 = hist_list[[1]]
p5 = hist_list[[2]]
p6 = hist_list[[3]]

p13 = pos_list[[1]]
p14 = pos_list[[2]]
p15 = pos_list[[3]]


#### Figure 3 ####
##### Slope + Histogram + Posterior #####
# Create a grid of plots
p = cowplot::plot_grid(
  p1, p2, p3, p4, p5, p6, p13, p14, p15,
  labels = c("(a)", "(b)", "(c)", "(d)", "(e)", "(f)", "(g)", "(h)", "(i)"), # Add labels to each plot
  label_size = 18,            # Set the font size for labels
  label_x = -0.03,            # Adjust the x position of labels
  label_y = 1.03,             # Adjust the y position of labels
  nrow = 3, ncol = 3,         # Set the number of rows and columns in the grid
  align = "hv",               # Align plots horizontally and vertically
  rel_heights = c(1, 1, 1.5)  # Set the relative heights for each row
)

# Apply theme adjustments
p = p + theme(
  plot.margin = margin(15, 10, 10, 10, unit = "mm"), # Adjust margins around the plot grid
  plot.background = element_rect(fill = "white", color = NA) # Set the background color to white
)

# Display the grid of plots
print(p)

ggsave(plot=p, filename=sprintf("output/figure/fig_03_slope_hist_posteriors_%s_%02d_sec.png", model_name, sec), width=13, height=12.7, dpi=600)
ggsave(plot=p, filename=sprintf("output/figure/fig_03_slope_hist_posteriors_%s_%02d_sec.svg", model_name, sec), width=13, height=12.7)

save_dir = "C:/Users/ryoma/D/writing/00-first/005_Otsuka_202x_MEE_umineko_playback/otsuka-umineko-playback/illustration/export"
ggsave(plot=p, filename=sprintf("%s/fig_03_slope_hist_posteriors_%s_%02d_sec.png", save_dir, model_name, sec), width=13, height=12.7, dpi=600)
ggsave(plot=p, filename=sprintf("%s/fig_03_slope_hist_posteriors_%s_%02d_sec.svg", save_dir, model_name, sec), width=13, height=12.7)


# remove all objects
print("Saved")
remove(list = ls())
