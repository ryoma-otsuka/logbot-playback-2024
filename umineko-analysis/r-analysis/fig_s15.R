source("src/utils.R")
setup()

#### Configuration ####
sec = 5
data_type_list = c("s_vedba", "ad_speed", "ad_mpr")
model_name_list = c("lmm", "lmm_st2.2")

pred_list = list()
p_index = 1

for (j in 1:length(model_name_list))
{
  model_name = model_name_list[[j]]
  
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
    
    #### Load LMM model ####
    fname_base = sprintf("%s_%s_%02ds", model_name, data_type, sec)
    model_output_file = sprintf("output/model-output/%s.RData", fname_base)
    print(model_output_file)
    fit = readRDS(file = model_output_file)
    # print(fit)
    
    #### Predictive checks ####
    if (data_type == "s_vedba")
    {
      title = "S-VeDBA"
      x_lim = c(-0.5, 0.5)
    } else if (data_type == "ad_speed")
    {
      title = "AD-Speed"
      x_lim = c(-3.0, 3.0)
    } else if (data_type == "ad_mpr")
    {
      title = "AD-MPR"
      x_lim = c(-0.03, 0.03)
    }
    pred = vis_posterior_predictive_check(fit, Y, title, I=46)
    pred = pred +
      xlim(x_lim) +
      theme(axis.title.y = element_text(margin = margin(r = 30))) +
      theme(plot.margin = unit(c(5, 10, 5, 5), "mm")) +
      NULL
    pred_list[[p_index]] = pred
    p_index = p_index + 1
  }
}

p1 = pred_list[[1]]
p2 = pred_list[[2]]
p3 = pred_list[[3]]
p4 = pred_list[[4]]
p5 = pred_list[[5]]
p6 = pred_list[[6]]

#### Figure S15 ####
# Create a grid of plots
p = cowplot::plot_grid(
  p1, p2, p3, p4, p5, p6,
  labels = c("(a)", "(b)", "(c)", "(d)", "(e)", "(f)"),  # Add labels to each plot
  label_size = 18,                    # Set the font size for labels
  label_x = -0.03,                    # Adjust the x position of labels
  label_y = 1.03,                     # Adjust the y position of labels
  nrow = 2, ncol = 3,                 # Set the number of rows and columns in the grid
  align = "hv",                       # Align plots horizontally and vertically
  rel_heights = c(1, 1)               # Set the relative heights for each row
)

# Apply theme adjustments
p = p + theme(
  plot.margin = margin(7, 5, 5, 5, unit = "mm"),  # Adjust margins around the plot grid
  plot.background = element_rect(fill = "white", color = NA) # Set the background color to white
)

ggsave(plot=p, filename=sprintf("output/figure/fig_s15_pred_checks_%02d_sec.png", sec), width=13, height=8, dpi=600)
ggsave(plot=p, filename=sprintf("output/figure/fig_s15_pred_checks_%02d_sec.svg", sec), width=13, height=8)

ggsave(plot=p, filename=sprintf("../output/figure-for-paper/png/fig_s15_pred_checks_%02d_sec.png", sec), width=13, height=8, dpi=350)
ggsave(plot=p, filename=sprintf("../output/figure-for-paper/svg/fig_s15_pred_checks_%02d_sec.svg", sec), width=13, height=8, dpi=600)

# remove all objects
print("Saved")
remove(list = ls())
