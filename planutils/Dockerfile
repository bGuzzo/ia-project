FROM aiplanning/planutils:latest

# Install linux useful tools
RUN apt-get update
RUN apt-get install nano
# Install solvers and tools
RUN planutils install -y val
RUN planutils install -y planning.domains
RUN planutils install -y popf
RUN planutils install -y optic
RUN planutils install -y smtplan
# Install the planner we will use
RUN planutils install -y lpg-td

