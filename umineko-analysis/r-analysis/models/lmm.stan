// Linear Mixed-Effects Model
// Random intercept for each cluster (Group ID) | Reparametrization

data {
  int<lower=0> N;                       // Sample size
  int<lower=0> P;                       // Number of predictors (intercept + covariates)
  int<lower=0> K;                       // Number of Group IDs
  vector[N] Y;                          // Variate (response variable)
  matrix[N, P] X;                       // Design matrix without a column (1s for intercept)
  int<lower=0, upper=K> GID[N];         // Group ID
  int<lower=0> N_new;                   // Sample size
  matrix[N_new, P] X_new;               // Design matrix without a column (1s for intercept)
  int<lower=0, upper=K> GID_new[N_new]; // Group ID
}

parameters {
  vector[P] beta;        // slope parameters (regression coefficients)
  real alpha;            // intercept parameter
  vector[K] z;           // Standard normal variables for random effects
  real<lower=0> sigma_u; // Hyper parameter
  real<lower=0> sigma_Y; // SD of normal distribution
}

transformed parameters {
  vector[K] u = sigma_u * z;      // Non-centered parameterization
  vector[N] mu;                   // Mean of normal distribution
  mu = X * beta + alpha + u[GID]; // Linear predictor
}

model {
  // Priors
  beta ~ normal(0, 5);    // Weakly informative prior
  alpha ~ normal(0, 5);   // Weakly informative prior
  sigma_u ~ normal(0, 3); // Weakly informative prior | Half-Normal Distribution
  sigma_Y ~ normal(0, 3); // Weakly informative prior | Half-Normal Distribution
  
  // Likelihood
  z ~ normal(0, 1);
  Y ~ normal(mu, sigma_Y);
}

generated quantities {
  vector[N] Y_pred;         // Simulated data from posterior for prediction check
  vector[N_new] Y_pred_new; // Simulated data from posterior for prediction check
  vector[N] log_lik;        // Log likelihood for model comparison using WAIC and PSIS-LOOCV
  
  for (i in 1:N)
    Y_pred[i] = normal_rng(mu[i], sigma_Y);
    
  for (i in 1:N_new) {
    vector[K] u_new;
    u_new[GID_new[i]] = sigma_u * normal_rng(0, 1); 
    Y_pred_new[i] = normal_rng(X_new[i] * beta + alpha + u_new[GID_new[i]], sigma_Y);
  }
  
  for (i in 1:N)
    log_lik[i] = normal_lpdf(Y[i] | mu[i], sigma_Y);
}
