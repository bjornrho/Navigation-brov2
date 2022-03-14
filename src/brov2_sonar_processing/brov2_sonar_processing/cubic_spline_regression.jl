using SmoothingSplines
using Distributions, Random, PGFPlotsX

rng = MersenneTwister(1234);

# Defining distributions
d = Rayleigh(80)
x = range(0, 500; length = 100)
pdf_original = pdf.(d, x)*140*10^2
pdf_noise = pdf_original + randn!(rng, zeros(100))*10^2

# Fitting and predicting
spl = fit(SmoothingSpline, x, pdf_noise, 150000.0)
pdf_pred = predict(spl) # fitted vector

pdf_normalized = pdf_noise ./ pdf_pred

# Plotting
p = @pgf Axis({ xlabel = "x", ylabel = "pdf" }, 
         PGFPlotsX.Plot({thick, yellow }, Table(x, pdf_original)),
         PGFPlotsX.Plot({thick, blue }, Table(x, pdf_noise)), 
         PGFPlotsX.Plot({thick, red }, Table(x, pdf_pred)),
         PGFPlotsX.Plot({thick, black }, Table(x, pdf_normalized)))
display(p)

