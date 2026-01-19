function beta_dot = FOGM_bias(beta, tau, sigma)

    eta = sigma * randn();

    beta_dot = -1/tau * beta + eta;

end