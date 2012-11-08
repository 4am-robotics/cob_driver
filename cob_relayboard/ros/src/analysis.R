library(e1071)

filename <- 'desire_LIB_01.csv'
A <- read.csv(filename)
A <- A[ -nrow(A), ]
colnames(A) <- c('time','voltage')
# A[,1] <- A[,1] / 60.0
A[,2] <- A[,2] / 1000.0

A$group = cut(A$time, breaks = seq(0, max(A$time), by=60))

A.split <- split(A, A$group)
A.split <- lapply(A.split, function(X) data.frame(time=mean(X$time), voltage=median(X$voltage)))
B <- do.call(rbind, A.split)

m <- svm(voltage ~ time, data = B)

B.predict <- data.frame( time = seq(min(B$time), max(B$time), length=1000) )
B.predict$voltage <- predict(m, B.predict)

B.predict$time <- B.predict$time / 60
B$time <- B$time / 60
A$time <- A$time / 60

y.limits <- c(quantile(A[,2], 0.001) - 0.1, quantile(A[,2], 0.999) + 0.1)
pdf(paste(filename, '.pdf',sep=''), width=12, height=7)
plot(A[,1], A[,2], type='p', xlab='Time [min]', ylab='Voltage [V]', main=paste("Battery discharge (", filename, ")", sep=""), ylim=y.limits)
points(B$time, B$voltage, col='green')
lines(B.predict$time, B.predict$voltage, col='red')
grid(col='grey30')
dev.off()
