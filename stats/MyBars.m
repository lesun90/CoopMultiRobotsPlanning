function [] = MyBars(x, y, ystd, w, rgb, miny, parent)
n = length(y)
nx = length(x)

for k = 1 : 1 : n
h = fill([x(k) - 0.5*w, x(k) + 0.5 * w, x(k) + 0.5 * w, x(k) - 0.5 * w], [miny, miny, y(k), y(k)], rgb);
set(h, 'FaceColor', rgb, 'parent', parent);
line([x(k), x(k)], [y(k) - 0.5 * ystd(k), y(k) + 0.5 * ystd(k)], 'linewidth', 1.05, 'parent', parent);
end


end

