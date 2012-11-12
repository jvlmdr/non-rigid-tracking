function tracks = predator(image_dir, seeds, base_radius)

  base_diameter = 2 * base_radius + 1;
  num_points = numel(seeds);

  SIFT_SIZE_TO_SCALE = 1 / 4;

  % Generate tracks.
  for i = 1:num_points
    fprintf('%d / %d\n', i, num_points);

    close all;
    % Track each point using TLD.
    seed = seeds(i);
    radius = seed.size * SIFT_SIZE_TO_SCALE * base_radius
    bbox = [seed.x - radius; seed.y - radius; ...
            seed.x + radius; seed.y + radius];

    opt.source = struct('camera', 0 ,'input', image_dir, 'bb0', bbox);
    opt.output = '_output/';
    min_win = base_radius;
    patchsize = 2 * [base_diameter, base_diameter];
    fliplr = 0;
    maxbbox = 1;
    update_detector = 1;
    opt.plot = struct('pex', 1, 'nex', 1, 'dt', 1, 'confidence', 1, 'target', 1, ...
        'replace', 0, 'drawoutput', 3, 'draw', 0, 'pts', 1, 'help', 0, ...
        'patch_rescale', 1, 'save', 0);

    opt.model = struct('min_win', min_win, 'patchsize', patchsize, ...
        'fliplr', fliplr, 'ncc_thesame', 0.95, 'valid', 0.5, 'num_trees', 10, ...
        'num_features', 13, 'thr_fern', 0.5, 'thr_nn', 0.65, 'thr_nn_valid', 0.7);
    % Synthesis of positive examples during initialization
    opt.p_par_init = struct('num_closest', 10, 'num_warps', 20, 'noise', 5, ...
        'angle', 20, 'shift', 0.02, 'scale', 0.02);
    % Synthesis of positive examples during update
    opt.p_par_update = struct('num_closest', 10, 'num_warps', 10, 'noise', 5, ...
        'angle', 10, 'shift', 0.02, 'scale', 0.02);
    % Negative examples initialization/update
    opt.n_par = struct('overlap', 0.2, 'num_patches', 100); 
    opt.tracker = struct('occlusion', 10);
    opt.control = struct('maxbbox', maxbbox, 'update_detector', update_detector, ...
        'drop_img', 1, 'repeat', 1);

    tic;
    [bb, conf] = tldExample(opt);

    tracks(i).bb = bb;
  end
end
