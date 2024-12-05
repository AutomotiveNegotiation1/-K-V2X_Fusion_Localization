function [xmap,ymap]=axis_convert(xr, yr)

xmap = (3.09+xr*2.5/7.7)*628/20.6;
ymap = (1.27+yr*2.5/7.7)*1124/37.21;

