 ## Draw some nice visualizations from the kalman filter
## output file, and compute the state  covariance matrix. 
library(ggplot2)

outfile <-  "build/out.txt"
outfile <- read.table(outfile, sep="\t", header=FALSE)
colnames(outfile) <- c("ex", "ey", "evx", "evy",
                       "mx", "my",
                       "tx", "ty", "tvx", "tvy")
outfile <- data.frame(outfile)

N <- nrow(outfile)
rmse <- sqrt((1/N) * unlist(Map(function(x) { sum(x^2) },
                                cbind(outfile['ex'] - outfile['tx'],
                                      outfile['ey'] - outfile['ty'],
                                      outfile['evx'] - outfile['tvx'],
                                      outfile['evy'] - outfile['tvy']))))
print(rmse)
pl.shapes <- c("e"=16,      "m"=17,     "t"=15)
pl.colors <- c("e"="black", "m"="blue", "t"="firebrick")
pl.alpha  <- c("e"=1,       "m"=0.5,    "t"=0.5)

pl.data <- do.call(rbind,
                   Map(function(tt) {
                     x = paste(tt, "x", sep="")
                     y = paste(tt, "y", sep="")                     
                     data.frame("x" = outfile[,x],
                                "y" = outfile[,y],
                                "tt"= tt)
                   }, c("e", "m", "t")))

head(pl.data)

pl <- ggplot(data=pl.data) +
  geom_point(aes(x, y, color=tt, shape=tt,  alpha=tt), size=5) +  
  scale_x_continuous() +
  scale_y_continuous() +
  scale_color_manual(name="uniqueL", values=pl.colors, labels=c("Truth", "Measurement", "Prediction")) +
  scale_shape_manual(name="uniqueL", values=pl.shapes, labels=c("Truth", "Measurement", "Prediction")) +
  scale_alpha_manual(name="uniqueL", values=pl.alpha, labels=c("Truth", "Measurement", "Prediction")) +
  theme(axis.line = element_blank(),
        panel.grid.major = element_line(colour="lightgray"),
        panel.grid.minor = element_line(colour="lightgray"),
        panel.border = element_blank(),
        panel.background = element_rect(fill="white"),
        text = element_text(size=30),
        legend.title=element_blank(),
        legend.text = element_text(size=20),
        legend.background = element_blank(),
        legend.key = element_blank()) + 
  ggtitle("Extended Kalman Filter (position)") +
  annotate("text", label=paste("RMSE: [", paste(round(rmse, 3), collapse=", "),  "]", sep=" "),
           x=10, y=-4, size=10) + 
  labs(x="x position", y="y position")


cov(outfile[,1:4])
cov(outfile[,5:6])
cov(outfile[,7:10])

png("sample-data-output.png", width=1024, height=1024)
pl
dev.off()
