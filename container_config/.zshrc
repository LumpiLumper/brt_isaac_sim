
export ZSH="$HOME/.oh-my-zsh"

ZSH_THEME="af-magic"

plugins=(git zsh-autosuggestions zsh-syntax-highlighting sudo history)

source $ZSH/oh-my-zsh.sh

export FPATH="~/.config/eza/completions/zsh:$FPATH"

# Arrow keys → normal history navigation
bindkey '^[[A' up-line-or-history
bindkey '^[[B' down-line-or-history

# Alias
alias eza="eza --icons --group-directories-first --git"

# Use eza instead of ls
alias ls="eza --icons --group-directories-first"
alias ll="eza -lah --icons --group-directories-first"
alias la="eza -a --icons"
alias l="eza -1 --icons"

# Tree view (limited depth)
alias lt="eza --tree --level=3 --icons"
alias lta="eza --tree --level=3 -a --icons"

# Git-related (since you use git plugin, keep short)
alias gs="git status"
alias ga="git add"
alias gc="git commit"
alias gp="git push"
alias gl="git pull"

# Navigation
alias ..="cd .."
alias ...="cd ../.."
alias ....="cd ../../.."

# Safer file operations
alias cp="cp -i"
alias mv="mv -i"
alias rm="rm -i"

# batcat as cat replacement (Ubuntu naming)
alias cat="batcat --style=plain"
alias bat="batcat"

# Quick file preview
alias less="batcat"

# System shortcuts
alias cls="clear"
alias h="history"
alias j="jobs -l"

# Disk usage
alias du="du -h"
alias df="df -h"

# Show PATH nicely
alias path='echo -e ${PATH//:/\\n}'


sudo-command-line() {
    [[ -z $BUFFER ]] && zle up-history
    if [[ $BUFFER == sudo\ * ]]; then
        LBUFFER="${LBUFFER#sudo }"
    else
        LBUFFER="sudo $LBUFFER"
    fi
}
zle -N sudo-command-line
bindkey "^[s" sudo-command-line

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"