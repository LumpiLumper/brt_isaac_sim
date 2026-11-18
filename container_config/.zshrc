export ZSH="$HOME/.oh-my-zsh"
ZSH_THEME="robbyrussell"
plugins=(git zsh-autosuggestions zsh-syntax-highlighting)
HISTFILE="$HOME/.zsh_history"
HISTSIZE=10000
SAVEHIST=10000
setopt HIST_IGNORE_DUPS SHARE_HISTORY

source $ZSH/oh-my-zsh.sh

source /opt/ros/jazzy/setup.zsh
[ -f /home/ws/install/setup.zsh ] && source /home/ws/install/setup.zsh

alias ls='eza'
alias cat='batcat'
alias cb='cd /home/ws && colcon build --symlink-install'
# Tab-completion for ros2 / colcon
autoload -U bashcompinit && bashcompinit
command -v register-python-argcomplete >/dev/null && eval "$(register-python-argcomplete ros2)"
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh