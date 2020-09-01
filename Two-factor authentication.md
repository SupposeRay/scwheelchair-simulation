# Two-factor Authentication for Git Command Line

Whenever two-factor authentication is enabled, you might encouter issues with normal git commands, like `git clone`, `git pull`, because each time you will be required to log in your username and password, but after typing your username and password, you are likely to get the following prompt:

`Username for 'https://github.com': your_user_name`
`Password for 'https://your_user_name@github.com': `
`remote: Invalid username or password.`
`fatal: Authentication failed for 'https://github.com/your_user_name/repo_name.git/'`

## Why is this happening?

The answer can be found from the [GitHub Help Documentation](https://docs.github.com/en/github/authenticating-to-github/accessing-github-using-two-factor-authentication#when-youll-be-asked-for-a-personal-access-token-as-a-password).

## How to fix it?

1. Generate a [Personal Access Token](https://github.com/settings/tokens). (Detailed guide on [Creating a personal access token for the command line](https://docs.github.com/en/github/authenticating-to-github/creating-a-personal-access-token).)

2. Copy the Personal Access Token.

3. Go back to the terminal and save the authentication of git. (Otherwise, you will be asked to log in your account with the Personal Access Token each time you use git commands)

   run in terminal: `git config --global credential.helper store`

4. Re-attempt the command you were trying and use Personal Access Token in the place of your password.
