library(CausalImpact)
library(ggplot2)
# source("src/ci_plot.R")

debug_mode = FALSE
# debug_mode = TRUE

# run_id = "run-05"
run_id = "run-10"
# run_id = "run-15"
# run_id = "run-20"

data_type_list = c("smoothed_VeDBA_2s", "abs_diff_distance_m", "abs_diff_pixel_count_p")
data_name_list = c("s_vedba", "ad_speed", "ad_mpr")
data_title_list = c("S-VeDBA", "AD-Speed", "AD-MPR")

# target_data = 0 # All
# target_data = 1 # S-VeDBA
# target_data = 2 # AD-Speed
target_data = 3 # AD-MPR

if (target_data > 0)
{
  data_type_list = c(data_type_list[target_data])
  data_name_list = c(data_name_list[target_data])
  data_title_list = c(data_title_list[target_data])
}

# plot_list = c()
# p_count = 1

for (i in (1:length(data_type_list)))
{
  data_type = data_type_list[[i]]
  data_name = data_name_list[[i]]
  
  parent_dir = sprintf("data/ci-data/%s/%s", run_id, data_type)
  session_paths = Sys.glob(file.path(parent_dir, "*"))
  session_name_list = basename(session_paths[file.info(session_paths)$isdir])
  if (debug_mode == TRUE)
  {
    session_name_list = c("LBP01_S00", "LBP03_S00") # Run only two sessions
  }
  print(session_name_list)
  
  for (j in (1:length(session_name_list)))
  {
    session_name = session_name_list[[j]]
    base_dir = sprintf("data/ci-data/%s/%s/%s", run_id, data_type, session_name)
    print(base_dir)
    
    # data
    data_path = sprintf("%s/data.csv", base_dir)
    data = read.csv(data_path)
    
    # periods
    periods_path = sprintf("%s/periods.csv", base_dir)
    periods = read.csv(periods_path)
    pre_period = periods$pre_period
    post_period = periods$post_period
    
    if (post_period[1] == 0)
    {
      print("Audio playback cancelled. -> Skip the data.")
      next # skip
    }
    
    # Python index start from zero while R index start from 1
    pre_period = pre_period + c(1, 1)
    post_period = post_period + c(1, 1)
    
    # check
    # print(head(data))
    # print(pre_period)
    # print(post_period)
    
    # check if the pre data is all zeros or not
    vec = as.vector(data$y[pre_period[1]:pre_period[2]])
    all_zero = all(vec == 0)
    print(sprintf("all_zero?: %s", all_zero))
    # add very small value if necessary
    if (all_zero)
    {
      set.seed(558)
      random_pre_index = sample(pre_period[1]:pre_period[2], 1)
      very_small_value = 0.0000001 # 1e-7
      data$y[random_pre_index] = data$y[random_pre_index] + very_small_value
    }
    
    # Run CausalImpact
    impact = CausalImpact(data, pre_period, post_period, alpha=0.03)
    
    # Save results
    output_save_dir = sprintf("../output/causal-impact/%s/%s/%s/r/", run_id, data_type, session_name)
    if (!dir.exists(output_save_dir)) {
      dir.create(output_save_dir, recursive = TRUE)
      cat("Directory created:", output_save_dir, "\n")
    } else {
      cat("Directory already exists:", output_save_dir, "\n")
    }
    write.csv(impact$series, file=sprintf("%s/df_ci_r.csv", output_save_dir))
    write.csv(impact$summary, file=sprintf("%s/summary_data_r.csv", output_save_dir))
    
    # Plot the results and save
    # .session_name = gsub("_", " ", session_name)
    # data_title = data_title_list[[i]]
    # # p = plot(impact)
    # p = plot_causal_impact(impact, title=sprintf("%s | %s", .session_name, data_title))
    # p = p +
    #   theme(axis.title.y = element_text(margin = margin(r = 5))) +
    #   theme(plot.margin = unit(c(5, 20, 5, 5), "mm")) +
    #   NULL
    # # print(p)
    # ggsave(plot=p, filename=sprintf("output/ci-figure/ci_%s_%s.png", session_name, data_name), width=5, height=5, dpi=350)
    # ggsave(plot=p, filename=sprintf("output/ci-figure/ci_%s_%s.svg", session_name, data_name), width=5, height=5)
    
    # plot_list[[p_count]] = p
    # p_count = p_count + 1
    
    # Show summary
    # summary(impact)
    # print(i)
    # print(j)
  }
}

# remove all objects
remove(list = ls())
