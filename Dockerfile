# Isaac Sim dev image: official Isaac Sim image + a workspace for this repo.
# The repo itself is bind-mounted at /workspace at run time (see docker-compose.yml),
# so edits made on the host are visible in the container immediately.
ARG ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1
FROM ${ISAAC_SIM_IMAGE}

ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    OMNI_KIT_ACCEPT_EULA=YES \
    PYTHONUNBUFFERED=1



USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        zsh \
        git \
        vim \
        curl \
        ca-certificates \
        eza \
        bat \
        && rm -rf /var/lib/apt/lists/*
    
USER isaac-sim

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
 && git clone --depth 1 https://github.com/zsh-users/zsh-autosuggestions \
      ${ZSH_CUSTOM:-/isaac-sim/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
 && git clone --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting \
      ${ZSH_CUSTOM:-/isaac-sim/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
 && sed -i 's/^plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' /isaac-sim/.zshrc

# Isaac Sim's bundled python (/isaac-sim/python.sh) is used to run scripts.
# Add extra pip packages your scripts need here, e.g.:
# RUN /isaac-sim/python.sh -m pip install --no-cache-dir open3d

WORKDIR /workspace

# Clear any entrypoint the base image sets so `docker compose run sim <cmd>` runs <cmd>.
ENTRYPOINT []
CMD ["zsh"]
