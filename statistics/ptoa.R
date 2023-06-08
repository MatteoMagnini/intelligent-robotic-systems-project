# set work directory 
# setwd("~/")
# setwd("C:/Users/Matteo/Desktop/Personale/Università/Magistrale/SIR/Progetto/statistics")

filename_Q <- "ptoa-Q"
filename_classic <- "ptoa-classic"
file_format <- ".csv"

# for a given file containing the positions of the stopped robots calculate the score.
# score is defined as the ratio between the first largest cluster and the second one.
distances_Q <- read.table(paste(filename_Q,file_format, sep=""), sep = ',')$V1
distances_classic <- read.table(paste(filename_classic,file_format,sep=""), sep = ',')$V1


plot(seq(1,length(distances_Q)),sort(distances_Q),col="blue",lwd = 2,
     main = "PTOA comparison",
     ylab = "Distance from light source",
     xlab = paste(as.character(length(distances_Q)),"experiments"))
points(seq(1,length(distances_classic)),sort(distances_classic),col="green",lwd = 2)
legend("topleft", legend=c("Q learning", "Handcrafted"),
       col=c("blue", "green"), lty=1:2, cex=0.8)


compare_distances_boxPlot <- function(distances_Q, distances_classic){
  
  # assuming length(distances_Q) == length(distances_classic)
  
  boxplot(distances_Q,distances_classic,
          main = "PTOA boxplot comparison",
          ylab = "Distance from light source",
          xlab = paste(as.character(length(distances_Q)),"experiments"),
          names = c("Q learning", "Handcrafted"),
          col = c("blue","green"))
  
}

compare_distances_boxPlot(distances_Q, distances_classic)

###########################################################################################

filename_circuit = "circuit"

metric_circuit <- read.table(paste(filename_circuit,file_format, sep=""), sep = ',')$V1

plot(seq(1,length(metric_circuit)),sort(metric_circuit,decreasing = TRUE),col="blue",lwd = 2,
     main = "Circuit Metric",
     ylab = "On circuit states over all states",
     xlab = paste(as.character(length(metric_circuit)),"experiments"))

boxplot(metric_circuit,
        main = "Circuit Metric boxplot",
        ylab = "On circuit states over all states",
        xlab = paste(as.character(length(metric_circuit)),"experiments"),
        col = "blue")

###########################################################################################


filename_oa = "obstacle-avoidance"

metric_oa <- read.table(paste(filename_oa,file_format, sep=""), sep = ',')$V1

plot(seq(1,length(metric_oa)),sort(metric_oa,decreasing = TRUE),col="blue",lwd = 2,
     main = "Obstacle Avoidance Metric",
     ylab = "Free obstacle states over all states",
     xlab = paste(as.character(length(metric_oa)),"experiments"))

boxplot(metric_oa,
        main = "Obstacle Avoidance metric boxplot",
        ylab = "Free obstacle states over all states",
        xlab = paste(as.character(length(metric_oa)),"experiments"),
        col = "blue")

###########################################################################################

filename_pt = "phototaxis"

metric_pt <- read.table(paste(filename_pt,file_format, sep=""), sep = ',')$V1

plot(seq(1,length(metric_pt)),sort(metric_pt),col="blue",lwd = 2,
     main = "Phototaxis Metric",
     ylab = "Distance from light source ",
     xlab = paste(as.character(length(metric_pt)),"experiments"))

boxplot(metric_pt,
        main = "Phototaxis metric boxplot",
        ylab = "Distance from light source",
        xlab = paste(as.character(length(metric_pt)),"experiments"),
        col = "blue")
