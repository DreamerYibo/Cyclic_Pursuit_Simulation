function x = convertn1p1pi(theta)
    %%Convert rad to [0, 2pi)
    while (any(theta<-pi, 'all') || any(theta >= pi, 'all'))
        theta(theta<-pi) = theta(theta<-pi) + 2*pi;
        theta(theta>= pi) = theta(theta>= pi) - 2*pi;
    end
    x = theta;
    end