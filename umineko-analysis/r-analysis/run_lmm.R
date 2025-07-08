# Run MCMC sampling using RStan and save the stan fit object
source("src/utils.R")
setup()

#### Configuration ####
sec = 5
# sec = 10

# data_type_list = c("s_vedba", "ad_speed", "ad_mpr")
# data_type_list = c("s_vedba")
# data_type_list = c("ad_speed")
data_type_list = c("ad_mpr")

model_name = "lmm"
# model_name = "lmm_st2.2"

#### Compile the Stan model ####
stan_file = sprintf("models/%s.stan", model_name)
stan_model = rstan::stan_model(file = stan_file)


for (i in 1:length(data_type_list)) 
{
  # data type
  data_type = data_type_list[[i]]
  
  #### Load data ####
  fname_base = sprintf("%s_%02ds", data_type, sec)
  path = sprintf("data/mean/%s.csv", fname_base)
  print(path)
  df = read.csv(path)
  df = subset(df, audio_file_name != "Cancelled") # filtering
  # scaling
  df$pb_count = scale(df$pb_count)
  df$pb_count_2 = scale(df$pb_count_2)
  df$body_mass = scale(df$body_mass)
  
  #### Response variable ####
  Y = as.vector(df$diff_value)
  
  #### Explanatory variables ####
  X_tmp = model.matrix(
    object = ~ audio_file_name + location2 + body_mass + pb_count_2, data = df
  )
  X = X_tmp[, -c(1)] # remove a column with 1s for intercept
  # head(X) # predator:  1 | Noise: 0
  
  #### Group ID (for random intercept) ####
  group_str = as.vector(df$test_id)
  unique_elements = unique(group_str)
  index_mapping = setNames(seq_along(unique_elements), unique_elements)
  group_id = index_mapping[group_str]
  # print(group)
  # print(str(group))
  N = nrow(df)  # sample size
  P = ncol(X)   # Number of explanatory variables
  K = length(unique(df$test_id))   # Number of groups
  
  #### New data ####
  N_new = 100 # Number of new data points
  X3_min_val = min(X[, 3])
  X3_max_val = max(X[, 3])
  X4_min_val = min(X[, 4])
  X4_max_val = max(X[, 4])
  X_new = matrix(NA, nrow = N_new, ncol = P)
  X_new[, 1] = sample(0:1, N_new, replace = TRUE) # binary
  X_new[, 2] = sample(0:1, N_new, replace = TRUE) # binary
  X_new[, 3] = runif(N_new, min = X3_min_val, max = X3_max_val) # real
  X_new[, 4] = runif(N_new, min = X4_min_val, max = X4_max_val) # real
  group_new = sample(1:K, N_new, replace = TRUE) # int
  
  #### Stan data ####
  stan_data = list(
    N = N, 
    P = P, 
    X = X, 
    Y = Y, 
    K = K, 
    GID = group_id, 
    N_new = N_new, 
    X_new = X_new,
    GID_new = group_new
  )
  print(str(stan_data))
  print(X)
  
  #### MCMC sampling ####
  fit = rstan::sampling(
    object = stan_model, 
    data = stan_data, 
    seed = 1234,
    chains = 4, 
    iter = 8000,
    warmup = 6000, 
    thin = 1,
    control = list(adapt_delta=0.99)
  )
  
  if (model_name == "lm")
  {
    params = c("alpha", "beta", "sigma_Y", "lp__")
  } else
  {
    params = c("alpha", "beta", "u", "sigma_u", "sigma_Y", "lp__")
  }
  print(fit, pars = params, probs = c(0.015, 0.055, 0.500, 0.945, 0.985), digits_summary = 3)
  
  #### Save model output ####
  model_output_file = sprintf("output/model-output/%s_%s.RData", model_name, fname_base)
  print(model_output_file)
  saveRDS(object = fit, file = model_output_file)
}

# remove all objects
remove(list = ls())
