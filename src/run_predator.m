function run_predator(image_dir, seeds_file, tracks_file, num_cores)

  RADIUS = 5;
  DIAMETER = 2 * RADIUS + 1;

  % Load seeds from file.
  data = cv.FileStorage(seeds_file);
  num_points = length(data.list);
  for i = 1:num_points
    point = data.list{i};
    seeds(i) = struct('x', point.x, 'y', point.y, 'size', point.size, ...
        'angle', point.angle);
  end

  % Run the algorithm.
  %results = predator(image_dir, seeds, RADIUS);
  results = parallelize(@(x) predator(image_dir, x, RADIUS), seeds, ...
      num_points, num_cores);
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
