function [] = MyBar(x, ymin, ymax, ystd, w, rgb, parent, which, ylimit)
if which == 1 || which == 0 || which == 3
h = fill([x - 0.5*w, x + 0.5 * w, x + 0.5 * w, x - 0.5 * w], [ymin, ymin, ymax, ymax], rgb);
set(h, 'FaceColor', rgb, 'parent', parent);
end
if which == 2 || which == 0
line([x, x], [ymax - 0.5 * ystd, ymax + 0.5 * ystd], 'linewidth', 1.05, 'parent', parent);
end
if which == 3
line([x, x], [ymax, ymax + 0.5 * ystd], 'linewidth', 1.05, 'parent', parent);
end
if ymax >= ylimit
    text(x, ylimit, 'X', 'parent', parent, ... 
	     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center', ...
        'fontsize', 9, 'fontname', 'Helvetica');
end

end

