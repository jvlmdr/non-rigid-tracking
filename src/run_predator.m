function run_predator(image_dir, seeds_file, tracks_file, num_cores)

  RADIUS = 5;
  DIAMETER = 2 * RADIUS + 1;

  SIFT_SIZE_TO_SCALE = 1 / 4;

  % Load seeds from file.
  data = cv.FileStorage(seeds_file);
  num_points = length(data.list);
  for i = 1:num_points
    point = data.list{i};
    if isfield(point, 'x') && isfield(point, 'y')
      if isfield(point, 'size')
        if isfield(point, 'angle')
          seeds(i) = struct('x', point.x, ...
                            'y', point.y, ...
                            'size', point.size * SIFT_SIZE_TO_SCALE);
        else
          seeds(i) = struct('x', point.x, ...
                            'y', point.y, ...
                            'size', point.size);
        end
      else
        seeds(i) = struct('x', point.x, 'y', point.y, 'size', 1);
      end
    end
  end

  % Run the algorithm.
  if exist('parallelize', 'file')
    results = parallelize(@(x) predator(image_dir, x, RADIUS), seeds, ...
        num_points, num_cores);
  else
    warning('Could not find parallelize');
    results = predator(image_dir, seeds, RADIUS);
  end
  save('results', 'results');

  % Save results.
  num_frames = size(results(1).bb, 2);

  tracks = [];
  for i = 1:num_points
    track = [];
    track.list = [];

    for t = 1:num_frames
      bbox = results(i).bb(:, t);

      if ~any(isnan(bbox))
        center = bb_center(bbox);

        point = [];
        point.x = center(1);
        point.y = center(2);
        point.size = bb_scale(bbox) / DIAMETER;

        frame = [];
        frame.t = int32(t - 1);
        frame.point = point;

        track.list = [track.list {frame}];
      end
    end
    tracks.list{i} = track;
  end

  cv.FileStorage(tracks_file, tracks);
end
