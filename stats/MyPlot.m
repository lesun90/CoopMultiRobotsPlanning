function [] = MyPlot(x, y, ystd, marker, rgb)
  markersize = 8;
  plot(x, y, marker, 'color', rgb, 'markersize', markersize, 'markerfacecolor', rgb, 'markeredgecolor', rgb, 'linewidth', 1.0);
  for k = 1 : 1 : length(ystd)
    line([x(k), x(k)], [y(k), y(k) + 0.5 * ystd(k)], 'color', [0 0 0], 'linewidth', 1.5);
  end

end