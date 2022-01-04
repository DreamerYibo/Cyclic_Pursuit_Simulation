function time_str = get_clock_str(time_raw)
    %%Convert clcok() array into a single str.
    N = length(time_raw);
    time_str = "";
    time = clock();
    time(N) = round(time(N));
    for i=1:N
        time_str = time_str+time(i);   
    end
end