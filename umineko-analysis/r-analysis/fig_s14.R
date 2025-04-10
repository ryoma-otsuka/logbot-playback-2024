source("src/utils.R")
setup()

#### Configuration ####
sec = 5
data_type_list = c("s_vedba", "ad_speed", "ad_mpr")
data_title_list = c("S-VeDBA", "AD-Speed", "AD-MPR")
model_name = "lmm"

dens_list = list()
post_list = list()

for (i in 1:length(data_type_list))
{
  data_type = data_type_list[[i]]
  data_title = data_title_list[[i]]
  
  #### Load LMM model ####
  fname_base = sprintf("%s_%s_%02ds", model_name, data_type, sec)
  model_output_file = sprintf("output/model-output/%s.RData", fname_base)
  fit = readRDS(file = model_output_file)
  # print(fit)
  
  parameters = c("alpha", "u", "sigma_u")
  p = vis_density(fit, parameters, ncol=5, inc_warmup=FALSE, title=data_title)
  p = p +
    theme(axis.title.y = element_text(margin = margin(r = 30))) +
    theme(plot.margin = unit(c(5, 10, 5, 5), "mm"))
  dens_list[[i]] = p
  
  # p = vis_randam_intercept_posteriors(fit, data_type)
  # p = p +
  #   theme(axis.title.y = element_text(margin = margin(r = 30))) +
  #   theme(plot.margin = unit(c(5, 10, 5, 5), "mm"))
  # print(p)
  # post_list[[i]] = p
  
}

p1 = dens_list[[1]]
p2 = dens_list[[2]]
p3 = dens_list[[3]]

# p4 = post_list[[1]]
# p5 = post_list[[2]]
# p6 = post_list[[3]]

### Figure S14 ####
# Create a grid of plots
p = cowplot::plot_grid(
  p1, p2, p3,
  labels = c("(a)", "(b)", "(c)"), # Add labels to each plot
  nrow = 3, ncol = 1,              # Set the number of rows and columns in the grid
  label_size = 18,                 # Set the font size for labels
  label_x = -0.01,                 # Adjust the x position of labels
  label_y = 1.01,                  # Adjust the y position of labels
  align = "hv",                    # Align plots horizontally and vertically
  rel_heights = c(1, 1, 1)         # Set the relative heights for each row
)

# Apply theme adjustments
p = p + theme(
  plot.margin = margin(10, 10, 10, 10, unit = "mm"), # Adjust margins around the plot grid
  plot.background = element_rect(fill = "white", color = NA) # Set the background color to white
)

ggsave(plot=p, filename=sprintf("output/figure/fig_s14_%s_%02d_sec.png", model_name, sec), width=13, height=14, dpi=600)
ggsave(plot=p, filename=sprintf("output/figure/fig_s14_%s_%02d_sec.svg", model_name, sec), width=13, height=14)

ggsave(plot=p, filename=sprintf("../output/figure-for-paper/png/fig_s14_%s_%02d_sec.png", model_name, sec), width=13, height=14, dpi=350)
ggsave(plot=p, filename=sprintf("../output/figure-for-paper/svg/fig_s14_%s_%02d_sec.svg", model_name, sec), width=13, height=14, dpi=350)

# remove all objects
print("Saved")
remove(list = ls())
