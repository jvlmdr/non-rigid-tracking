addpath(genpath('~/src/OpenTLD'));
tic;

RADIUS = 5;
DIAMETER = 2 * RADIUS + 1;

% Load seeds from file.
seeds = [];
addpath(genpath('~/src/yamlmatlab'));
data = ReadYaml('../release/seeds.yaml');
rmpath(genpath('~/src/yamlmatlab'));
num_points = length(data.list);
for i = 1:num_points
  point = data.list{i}.list{1}.point;
  seeds = [seeds; point.x, point.y];
end

% Generate tracks.
clear results;
for i = 1:num_points
  display(num2str(i));

  close all;
  % Track each point using TLD.
  seed = seeds(i, :);
  bbox = [seed(1) - RADIUS; seed(2) - RADIUS; ...
          seed(1) + RADIUS; seed(2) + RADIUS];
  opt.source = struct('camera', 0 ,'input', '../input/clap-subset-2-small/7/', ...
      'bb0', bbox);
  opt.output = '_output/';
  mkdir(opt.output);
  min_win = RADIUS;
  patchsize = [DIAMETER, DIAMETER];
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

  [bb, conf] = tldExample(opt);
  results(:, :, i) = bb;
end

% Save results.
num_frames = size(results, 2);

tracks = [];
for i = 1:num_points
  track = [];
  track.list = [];

  for t = 1:num_frames;
    bbox = results(:, t, i);

    if ~any(isnan(bbox))
      point = [];
      point.x = (bbox(3) + bbox(1)) / 2;
      point.y = (bbox(4) + bbox(2)) / 2;

      frame = [];
      frame.t = t;
      frame.point = point;

      track.list = [track.list {frame}];
    end
  end
  tracks.list{i} = track;
end

addpath(genpath('~/src/yamlmatlab'));
WriteYaml('predator-tracks.yaml', tracks);
rmpath(genpath('~/src/yamlmatlab'));
