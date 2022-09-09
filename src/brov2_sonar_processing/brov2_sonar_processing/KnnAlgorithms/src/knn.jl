module KnnAlgorithm

import NearestNeighbors
import Distances
import Statistics
import ImageFiltering
import Colors
import StatsBase
using DataFrames
using ColorSchemes

function anisotropic_diffusion(img; niter=1, kappa=50, gamma=0.1, voxelspacing=nothing, option=1)
    # define conduction gradients functions
    if option == 1
        condgradient(delta, spacing) = exp(-(delta/kappa)^2.)/spacing
    #elseif option == 2
    #    condgradient(delta, spacing) = 1.0/(1.0+(delta/kappa)^2.0)/Float64(spacing)
    #elseif option == 3
    #    kappa_s = kappa * (2**0.5)
    #    condgradient(delta, spacing) = ifelse(abs.(delta) .<= kappa_s, 0.5*((1.-(delta/kappa_s)**2.)**2.)/Float64(spacing), 0.0)
    end
    # initialize output array
    out = img

    # set default voxel spacing if not supplied
    if voxelspacing == nothing
        voxelspacing = ones(Float64, length(size(out)))
    end
    # initialize some internal variables
    deltas = [zeros(Float64,size(out)) for k ∈ 1:length(size(out))]

    for _k ∈ 1:niter

        # calculate the diffs
        for i ∈ 1:length(size(out))
            slicer = []
            for j ∈ 1:length(size(out))
                append!(slicer,j==i ? [[1:size(out)[j]-1...]] : [[1:size(out)[j]...]])
            end
            deltas[i][slicer...] = diff(out,dims=i)
        end
        # update matrices
        matrices = [condgradient(deltas[i], voxelspacing[i]) * deltas[i] for i ∈ 1:length(deltas)]
        # subtract a copy that has been shifted ('Up/North/West' in 3D case) by one
        # pixel. Don't as questions. just do it. trust me.
        for i ∈ 1:length(size(out))
            slicer = []
            for j ∈ 1:length(size(out))
                append!(slicer,j==i ? [[2:size(out)[j]...]] : [[1:size(out)[j]...]])
            end
            matrices[i][slicer...] = diff(matrices[i],dims=i)
        end
        # update the image
        out = out + gamma * sum(matrices)
    end
    return out
end

export knn

function knn(res, u, v, intensity_values)
    df = DataFrame((u=u, v=v, intensity=intensity_values))
    df = filter(:intensity => intensity -> !any(f -> f(intensity), (ismissing, isnothing, isnan)), df)

    datapoints = (Matrix(df[:,1:2])')
    datavalues = df[:,3]

    kdtree = NearestNeighbors.KDTree(datapoints, Distances.Euclidean())
    ugrid = minimum(datapoints[1,1:size(datapoints)[2]]):res:maximum(datapoints[1,1:size(datapoints)[2]])
    vgrid = minimum(datapoints[2,1:size(datapoints)[2]]):res:maximum(datapoints[2,1:size(datapoints)[2]])

    kNN_param = 4;
    variance_ceiling = 0.005
    intensity_mean = fill(NaN,length(ugrid),length(vgrid))
    intensity_variance = fill(NaN,length(ugrid),length(vgrid))

    for i=1:length(ugrid),j=1:length(vgrid)
        valid_distance = 0.3
        idx, dists= NearestNeighbors.knn(kdtree,[ugrid[i],vgrid[j]],kNN_param)
        svals = [datavalues[idx[ind]] for ind ∈ 1:kNN_param if dists[ind] <= valid_distance]

        if svals != []
            if Statistics.var(svals) <= variance_ceiling || svals != NaN == NaN
                intensity_mean[i,j] = Statistics.mean(svals)
                intensity_variance[i,j] = Statistics.var(svals)
            else
                intensity_mean[i,j] = StatsBase.percentile(svals, 10/100)
                intensity_variance[i,j] = variance_ceiling
            end
        end
    end;

    temp = copy(intensity_mean)
    n_dim = max(size(temp)[1], size(temp)[2])
    if size(temp)[1] < size(temp)[2] grid = vgrid else grid = ugrid end
    padded_intensity_mean = fill(NaN, n_dim,n_dim)
    padded_intensity_mean[1:size(temp)[1],1:size(temp)[2]] = temp
    mask = isnan.(padded_intensity_mean)
    replace!(padded_intensity_mean, NaN=>0.0)
    filtered_image = anisotropic_diffusion(padded_intensity_mean, kappa=20, gamma=0.25, option=1)
    filtered_image[mask] .= NaN;

    return (intensity_mean, intensity_variance, filtered_image)
end

precompile(knn, (Float64, Vector{Float64}, Vector{Float64},))

end